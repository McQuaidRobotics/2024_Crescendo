package com.igknighters.subsystems.stem;

import com.igknighters.constants.ConstValues.kStem.kPivot;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class StemSolvers {

    public static double gravitySolveWristTheta(
        double stemLength, 
        double pivotRads, 
        double horizDist, 
        double vertDist,
        double initialNoteVelo) {

        Translation2d wristLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        horizDist += wristLocation.getX();
        vertDist -= wristLocation.getY();

        double linearWristRads = linearSolveWristTheta(stemLength, pivotRads, horizDist, vertDist);
        double vertcialDistanceError = linearSolveVerticalDistError(linearWristRads, horizDist, vertDist, initialNoteVelo);

        double newVerticalDist = vertDist + vertcialDistanceError;

        return linearSolveWristTheta(stemLength, pivotRads, horizDist, newVerticalDist);
    }

    /**
     * Returns the theta of the wrist given the input parameters in radians.
     * 
     * @apiNote When using with the telescope enabled make sure to factor in the
     *          current exention of the telescope in meters into your stem length
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist
     *                   axel
     * @param pivotRads  The angle of the pivot in radians
     * @param horizDist  The horizontal distance from the pivot axel to the target
     * @param vertDist   The vertical distance from the pivot axel to the target
     * @return The angle of the wrist in radians
     */
    public static double linearSolveWristTheta(
            double stemLength,
            double pivotRads,
            double horizDist,
            double vertDist) {

        Translation2d wristLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        return Math.atan((vertDist - wristLocation.getY()) / (horizDist + wristLocation.getX()));
    }

    /**
     * Returns the theta of the pivot given the input parameters in radians.
     * 
     * @apiNote This is meant to be used with a variant of the stem without a mobile
     *          wrist
     *          as this solve is less accurate than a solve involving just varying
     *          the theta
     *          of the wrist at a known position.
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist
     *                   axel
     * @param wristRads  The angle of the wrist in radians
     * @param horizDist  The horizontal distance from the pivot axel to the target
     * @param vertDist   The vertical distance from the pivot axel to the target
     * @return The angle of the pivot in radians
     */
    public static double linearSolvePivotTheta(
            double stemLength,
            double wristRads,
            double horizDist,
            double vertDist) {

        horizDist += kPivot.kDimensions.PIVOT_AXEL_LOCATION.getX();
        vertDist -= kPivot.kDimensions.PIVOT_AXEL_LOCATION.getY();

        double kHyp = Math.sqrt(horizDist * horizDist + vertDist * vertDist);
        double aK = Math.asin(vertDist / kHyp);
        double aS = Math.asin((stemLength * Math.sin(wristRads)) / kHyp);
        double aP = Math.PI - wristRads - aS;
        return Math.PI - aK - aP;
    }

    /**
     * Calculates the difference between the vertical distance to the target and the
     * distance below the target that a note would land if shot with the given
     * parameters.
     * 
     * @param wristRads           The angle of the wrist in radians
     * @param noteInitialVelocity The initial velocity of the note
     * @param horizontalDist      The horizontal distance from the wrist axel to the
     *                            target
     * @param verticalDist        The vertical distance from the wrist axel to the
     *                            target
     * @return The difference between the vertical distance to the target and the
     *         distance below the target that a note would land if shot with the
     *         given parameters
     */
    public static double linearSolveVerticalDistError(
            double wristRads,
            double horizontalDist,
            double verticalDist,
            double noteInitialVelocity) {

            double horizontalVi = noteInitialVelocity * Math.cos(wristRads);
            double verticalVi = noteInitialVelocity * Math.sin(wristRads);
            double time = horizontalDist / horizontalVi;
            double newVerticalDist = (verticalVi * time) + (0.5 * -9.8 * Math.pow(time, 2));
            return Math.abs(verticalDist - newVerticalDist);
    }

    /**
     * Solves the for the location of wrist axel relative to the center of the robot in 2d space
     * 
     * @apiNote This can be thought of side view of the robot in cartesian space
     *          with the front of the robot being +X.
     * 
     * @param stemLength The length of the stem from pivot axel to the wrist axel
     * @param pivotRads The angle of the pivot in radians
     * @return The Translation2d representing the location of the wrist axel
     */
    public static Translation2d solveWristLocationSimple2d(double stemLength, double pivotRads) {
        Translation2d wristLocationVector = new Translation2d(
            stemLength * Math.cos(pivotRads),
            stemLength * Math.sin(pivotRads)
        );
        return kPivot.kDimensions.PIVOT_AXEL_LOCATION.plus(wristLocationVector);
    }

    /**
     * Solves for the location of the wrist axel in 3d space relative to the
     * center of the robot on the x and y axis and relative to the floor on the z
     * axis.
     * 
     * @param pivotTransform The transform of the pivot axel relative to the robot
     * @param pivotRads      The angle of the pivot in radians
     * @param stemLength     The length of the stem from pivot axel to wrist axel
     * 
     * @return The {@link Transform3d} representing the location of the wrist axel
     */
    public static Translation3d solveWristLocation3d(double stemLength, double pivotRads) {
        Translation2d wristAxelLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        return new Translation3d(
            wristAxelLocation.getX(),
            0.0,
            wristAxelLocation.getY()
        );
    }

    public static final class Advanced {
        public static double interativeSolveV1PivotTheta(
                double stemLength,
                double wristRads,
                double horizDist,
                double vertDist,
                double noteInitialVelocity,
                int iterations) {
            double lastPivotTheta = linearSolvePivotTheta(stemLength, wristRads, horizDist, vertDist);
            for (int i = 0; i < iterations; i++) {
                Translation2d wristLocation = solveWristLocationSimple2d(stemLength, lastPivotTheta);
                double verticalDistError = linearSolveVerticalDistError(
                        lastPivotTheta,
                        horizDist + wristLocation.getX(),
                        vertDist - wristLocation.getY(),
                        noteInitialVelocity);
                lastPivotTheta = linearSolvePivotTheta(stemLength, wristRads, horizDist, vertDist + verticalDistError);
            }

            return lastPivotTheta;
        }
    }
}
