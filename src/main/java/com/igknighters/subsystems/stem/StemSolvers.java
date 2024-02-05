package com.igknighters.subsystems.stem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class StemSolvers {

    /**
     * Returns the theta of the pivot given the input parameters in radians.
     * 
     * @apiNote This is mean to be used with a variant of the stem without a mobile wrist
     * as this solve is less accurate than a solve involving just varying the theta
     * of the wrist at a known position.
     * 
     * @param stemLength The length of the stem from pivot axel to where the note
     *                   leaves the shooter
     * @param wristAngle The angle of the wrist in radians
     * @param horizDist  The horizontal distance from the pivot axel to the target
     * @param vertDist   The vertical distance from the pivot axel to the target
     * @return The angle of the pivot in radians
     */
    public static double linearSolvePivotTheta(
            double stemLength,
            double wristAngle,
            double horizDist,
            double vertDist) {
        double kHyp = Math.sqrt(horizDist * horizDist + vertDist * vertDist);
        double aK = Math.asin(vertDist / kHyp);
        double aS = Math.asin((stemLength * Math.sin(wristAngle)) / kHyp);
        double aP = Math.PI - wristAngle - aS;
        return Math.PI - aK - aP;
    }

    /**
     * Calculates the difference between the vertical distance to the target and the
     * distance below the target that a note would land if shot with the given parameters.
     * 
     * @param endEffectorTheta The angle of the end effector in radians
     * @param noteInitialVelocity The initial velocity of the note
     * @param horizontalDist The horizontal distance from the wrist axel to the target
     * @param verticalDist The vertical distance from the wrist axel to the target
     * @return The difference between the vertical distance to the target and the
     *         distance below the target that a note would land if shot with the given parameters
     */
    public static double linearSolveVerticalDistError(
            double endEffectorTheta,
            double noteInitialVelocity,
            double horizontalDist,
            double verticalDist) {
        // double initialVerticalVelo = noteInitialVelocity * Math.sin(endEffectorTheta);
        double initialHorizontalVelo = noteInitialVelocity * Math.cos(endEffectorTheta);
        double time = (horizontalDist / initialHorizontalVelo);
        double distanceBelowTarget = (noteInitialVelocity * time) + (0.5 * -9.8 * (time * time));
        return verticalDist - distanceBelowTarget;
    }

    /**
     * Solves the for the location of wrist axel relative to the pivot axel
     * in 2d space.
     * 
     * @apiNote This can be thought of side view of the robot in cartesian space
     * with the pivot axel at the origin and the front of the robot being +X.
     * 
     * @param stemLength The length of the stem from pivot axel to where the note
     *                   leaves the shooter
     * @param pivotTheta The angle of the pivot in radians
     * @return The Transform2d representing the location of the wrist axel
     */
    public static Translation2d solveWristLocationSimple2d(
            double stemLength,
            double pivotTheta) {
        return new Translation2d(
                stemLength,
                new Rotation2d(pivotTheta)
        );
    }

    /**
     * Solves for the location of the wrist axel in 3d space relative to the
     * center of the robot on the x and y axis and relative to the floor on the z axis.
     * 
     * @param pivotTransform The transform of the pivot axel relative to the robot
     * @param pivotTheta The angle of the pivot in radians
     * @param stemLength The length of the stem from pivot axel to wrist axel
     * 
     * @return The {@link Transform3d} representing the location of the wrist axel
     */
    public static Transform3d solveRelativeWristLocation3d(
            Transform3d pivotTransform,
            double pivotTheta,
            double stemLength) {
        return pivotTransform.plus(new Transform3d(
                new Translation3d(
                    stemLength,
                    new Rotation3d(
                        0,
                        pivotTheta,
                        0
                    )
                ),
                new Rotation3d()
        ));
    }

    public static final class Advanced {
        public static double interativeSolveV1PivotTheta(
                double stemLength,
                double wristAngle,
                double horizDist,
                double vertDist,
                double noteInitialVelocity,
                int iterations
        ) {
            double lastPivotTheta = linearSolvePivotTheta(stemLength, wristAngle, horizDist, vertDist);
            for (int i = 0; i < iterations; i++) {
                Translation2d wristOffset = solveWristLocationSimple2d(stemLength, lastPivotTheta);
                double verticalDistError = linearSolveVerticalDistError(
                        lastPivotTheta,
                        noteInitialVelocity,
                        horizDist + wristOffset.getX(),
                        vertDist + wristOffset.getY()
                );
                lastPivotTheta = linearSolvePivotTheta(stemLength, wristAngle, horizDist, vertDist + verticalDistError);
            }

            return lastPivotTheta;
        }
    }
}
