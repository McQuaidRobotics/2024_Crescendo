package com.igknighters.subsystems.stem;

import java.util.function.Function;

import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableBoolean;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import monologue.Monologue;

public class StemSolvers {
    private static final TunableBoolean DEBUG_SOLVERS = TunableValues.getBoolean("Stem/DebugSolvers", false);
    /** Log Prefix */
    private static final String LP = "StemSolvers";

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
     * @param offsetPivot Whether or not to offset the pivot angle
     * @return The angle of the wrist in radians
     */
    public static double linearSolveWristTheta(
            double stemLength,
            double pivotRads,
            double horizDist,
            double vertDist,
            boolean offsetPivot) {

        Translation2d wristLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        double flatWristRads = Math.atan((vertDist - wristLocation.getY()) / (horizDist + wristLocation.getX()));

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/linearSolveWristTheta/input";
            final String out = LP + "/linearSolveWristTheta/output";

            Monologue.log(inp + "/stemLength", stemLength);
            Monologue.log(inp + "/pivotRads", pivotRads);
            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/vertDist", vertDist);
            Monologue.log(inp + "/offsetPivot", offsetPivot);

            Monologue.log(out + "/wristLocation", wristLocation);
            Monologue.log(out + "/flatWristRads", flatWristRads);
            Monologue.log(out + "/ret", offsetPivot ? flatWristRads + pivotRads : flatWristRads);
        }

        return offsetPivot ? flatWristRads + pivotRads : flatWristRads;
    }

    /**
     * Returns the theta of the wrist given the input parameters in radians.
     * 
     * @apiNote When using with the telescope enabled make sure to factor in the
     *          current exention of the telescope in meters into your stem length
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist axel
     * @param pivotRads The angle of the pivot in radians
     * @param horizDist The horizontal distance from the pivot axel to the target
     * @param noteVelo The average velocity of the note
     * @param time The time that the entire pass should take from start to finish
     * @param offsetPivot Whether or not to offset the pivot angle
     * @return The angle of the wrist in radians
     */
    public static double passWristSolveTheta(
        double stemLength, 
        double pivotRads, 
        double horizDist, 
        double noteVelo,
        double time, 
        boolean offsetPivot
    ) {

        Translation2d wristLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        double deltaXVelo = (horizDist + wristLocation.getX()) / time;
        double deltaYVelo = Math.sqrt(Math.pow(noteVelo, 2) - Math.pow(deltaXVelo, 2));
        double atan = Math.atan(deltaYVelo / deltaXVelo);

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/passWristSolveTheta/input";
            final String out = LP + "/passWristSolveTheta/output";

            Monologue.log(inp + "/stemLength", stemLength);
            Monologue.log(inp + "/pivotRads", pivotRads);
            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/noteVelo", noteVelo);
            Monologue.log(inp + "/time", time);
            Monologue.log(inp + "/offsetPivot", offsetPivot);

            Monologue.log(out + "/wristLocation", wristLocation);
            Monologue.log(out + "/deltaXVelo", deltaXVelo);
            Monologue.log(out + "/deltaYVelo", deltaYVelo);
            Monologue.log(out + "/atan", atan);
            Monologue.log(out + "/ret", offsetPivot ? atan + pivotRads : atan);
        }

        return offsetPivot ? atan + pivotRads : atan;
    }

    /**
     * Returns the theta of the wrist given the input parameters in radians.
     * 
     * @apiNote When using with the telescope enabled make sure to factor in the
     *          current exention of the telescope in meters into your stem length
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist axel
     * @param pivotRads The angle of the pivot in radians
     * @param robotLocation The location of the robot on the field
     * @param targetLocation The location of the target on the field
     * @param noteVelo The average velocity of the note
     * @param rising Whether or not to aim for the target at the apex of the notes flight
     * @param offsetPivot   Whether or not to offset the pivot angle
     * @return The angle of the wrist in radians
     */
    public static double gravitySolveWristTheta2(
        double stemLength,
        double pivotRads,
        Translation2d robotLocation,
        Translation3d targetLocation,
        double noteVelo,
        boolean rising,
        boolean offsetPivot
    ) {

        Translation2d wrist2dOffset = solveWristLocationSimple2d(stemLength, pivotRads);
        double offset = offsetPivot ? pivotRads : 0.0;

        double horizDist = targetLocation.toTranslation2d()
            .getDistance(robotLocation)
            + wrist2dOffset.getX();
        double vertDist = targetLocation.getZ() - wrist2dOffset.getY();

        double sqrFlightTime = Math.pow(horizDist, 2) / Math.pow(noteVelo, 2);
        double dropOverFlight = 9.8 * sqrFlightTime;
        double adjVertDist = vertDist + dropOverFlight;
        double flightDistance = Math.hypot(adjVertDist, horizDist);

        double theta;
        if (rising) {
            theta = Math.atan(adjVertDist / horizDist);
        } else {
            theta = Math.atan(adjVertDist / horizDist) + Math.acos(horizDist / flightDistance);
        }

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/gravitySolveWristTheta2/input";
            final String out = LP + "/gravitySolveWristTheta2/output";

            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/vertDist", vertDist);
            Monologue.log(inp + "/noteVelo", noteVelo);
            Monologue.log(inp + "/offsetPivot", offsetPivot);
            Monologue.log(inp + "/robotLocation", robotLocation);
            Monologue.log(inp + "/targetLocation", targetLocation);
            Monologue.log(inp + "/rising", rising);

            Monologue.log(out + "/wrist2dOffset", wrist2dOffset);
            Monologue.log(out + "/offset", offset);
            Monologue.log(out + "/sqrFlightTime", sqrFlightTime);
            Monologue.log(out + "/dropOverFlight", dropOverFlight);
            Monologue.log(out + "/flightDistance", flightDistance);
            Monologue.log(out + "/theta", theta);
            Monologue.log(out + "/ret", theta + offset);
        }

        return theta + offset;
    }

    /**
     * Returns the theta of the wrist given the input parameters in radians.
     * 
     * @apiNote When using with the telescope enabled make sure to factor in the
     *          current exention of the telescope in meters into your stem length
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist axel
     * @param pivotRads The angle of the pivot in radians
     * @param robotLocation The location of the robot on the field
     * @param targetLocation The location of the target on the field
     * @param noteVelo The average velocity of the note
     * @param rising Whether or not to aim for the target at the apex of the notes flight
     * @param offsetPivot   Whether or not to offset the pivot angle
     * @return The angle of the wrist in radians
     */
    public static double gravitySolveWristTheta3(
        double stemLength,
        double pivotRads,
        Translation2d robotLocation,
        Translation3d targetLocation,
        double noteVelo,
        boolean rising,
        boolean offsetPivot
    ) {

        Translation2d wrist2dOffset = solveWristLocationSimple2d(stemLength, pivotRads);
        double offset = offsetPivot ? pivotRads : 0.0;

        double horizDist = targetLocation.toTranslation2d()
            .getDistance(robotLocation)
            + wrist2dOffset.getX();
        double vertDist = wrist2dOffset.getY() - targetLocation.getZ();

        double sqrFlightTime = Math.pow(horizDist, 2) / Math.pow(noteVelo, 2);
        double dropOverFlight = 9.8 * sqrFlightTime;
        double flightDistance = Math.hypot(vertDist, horizDist);
        double adjVertDist = dropOverFlight - vertDist;

        double phi;
        double theta;
        double oppositeOverAdjacent = adjVertDist / flightDistance;
        if (rising) {
            phi = Math.atan(vertDist / horizDist);
            theta =  (Math.asin(oppositeOverAdjacent) - phi) / 2.0;
        } else {
            phi = Math.atan(horizDist / vertDist);
            theta = (Math.acos(oppositeOverAdjacent) + phi) / 2.0;
        }

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/gravitySolveWristTheta3/input";
            final String out = LP + "/gravitySolveWristTheta3/output";

            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/vertDist", vertDist);
            Monologue.log(inp + "/noteVelo", noteVelo);
            Monologue.log(inp + "/offsetPivot", offsetPivot);
            Monologue.log(inp + "/robotLocation", robotLocation);
            Monologue.log(inp + "/targetLocation", targetLocation);
            Monologue.log(inp + "/rising", rising);

            Monologue.log(out + "/wrist2dOffset", wrist2dOffset);
            Monologue.log(out + "/offset", offset);
            Monologue.log(out + "/sqrFlightTime", sqrFlightTime);
            Monologue.log(out + "/dropOverFlight", dropOverFlight);
            Monologue.log(out + "/flightDistance", flightDistance);
            Monologue.log(out + "/theta", theta);
            Monologue.log(out + "/ret", (-theta) + offset);
        }

        return theta + offset;
    }

    /**
     * Returns the theta of the pivot given the input parameters in radians.
     * 
     * @apiNote This is meant to be used with a variant of the stem without a mobile
     *          wrist as this solve is less accurate than a solve involving just varying
     *          the theta of the wrist at a known position.
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
            double vertDist
    ) {

        horizDist += kPivot.kDimensions.PIVOT_AXEL_LOCATION.getX();
        vertDist -= kPivot.kDimensions.PIVOT_AXEL_LOCATION.getY();

        double kHyp = Math.sqrt(horizDist * horizDist + vertDist * vertDist);
        double aK = Math.asin(vertDist / kHyp);
        double aS = Math.asin((stemLength * Math.sin(wristRads)) / kHyp);
        double aP = Math.PI - wristRads - aS;

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/linearSolvePivotTheta/input";
            final String out = LP + "/linearSolvePivotTheta/output";

            Monologue.log(inp + "/stemLength", stemLength);
            Monologue.log(inp + "/wristRads", wristRads);
            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/vertDist", vertDist);

            Monologue.log(out + "/kHyp", kHyp);
            Monologue.log(out + "/aK", aK);
            Monologue.log(out + "/aS", aS);
            Monologue.log(out + "/aP", aP);
            Monologue.log(out + "/ret", Math.PI - aK - aP);
        }

        return Math.PI - aK - aP;
    }

    /**
     * Calculates the difference between the vertical distance to the target and the
     * distance below the target that a note would land if shot with the given
     * parameters.
     * 
     * @param wristRads           The angle of the wrist in radians
     * @param deltaNoteVelo       The average velocity of the note
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
            double deltaNoteVelo
    ) {

        double horizontalVi = deltaNoteVelo * Math.cos(wristRads);
        double verticalVi = deltaNoteVelo * Math.sin(wristRads);
        double time = horizontalDist / horizontalVi;
        double newVerticalDist = (verticalVi * time) + (0.5 * -9.8 * Math.pow(time, 2));

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/linearSolveVerticalDistError/input";
            final String out = LP + "/linearSolveVerticalDistError/output";

            Monologue.log(inp + "/wristRads", wristRads);
            Monologue.log(inp + "/deltaNoteVelo", deltaNoteVelo);
            Monologue.log(inp + "/horizontalDist", horizontalDist);
            Monologue.log(inp + "/verticalDist", verticalDist);

            Monologue.log(out + "/horizontalVi", horizontalVi);
            Monologue.log(out + "/verticalVi", verticalVi);
            Monologue.log(out + "/time", time);
            Monologue.log(out + "/newVerticalDist", newVerticalDist);
        }

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

    /**
     * Returns the theta of the wrist with gravity compensation given the input parameters in radians.
     * 
     * @param stemLength The length of the stem from the pivot axel to the wrist
     * @param pivotRads The angle of the pivot in radians
     * @param horizDist The horizontal distance from the pivot axel to the target
     * @param vertDist The vertical distance from the pivot axel to the target
     * @param deltaNoteVelo The average velocity of the note
     * @return The angle of the wrist in radians
     */
    public static double gravitySolveWristTheta(
        double stemLength, 
        double pivotRads, 
        double horizDist, 
        double vertDist,
        double deltaNoteVelo) {

        Translation2d wristLocation = solveWristLocationSimple2d(stemLength, pivotRads);
        horizDist += wristLocation.getX();
        vertDist -= wristLocation.getY();

        double linearWristRads = linearSolveWristTheta(stemLength, pivotRads, horizDist, vertDist, false);
        double vertcialDistanceError = linearSolveVerticalDistError(linearWristRads, horizDist, vertDist, deltaNoteVelo);

        double newVerticalDist = vertDist + vertcialDistanceError;

        double grav = linearSolveWristTheta(stemLength, pivotRads, horizDist, newVerticalDist, true);

        if (DEBUG_SOLVERS.value()) {
            final String inp = LP + "/gravitySolveWristTheta/input";
            final String out = LP + "/gravitySolveWristTheta/output";

            Monologue.log(inp + "/stemLength", stemLength);
            Monologue.log(inp + "/pivotRads", pivotRads);
            Monologue.log(inp + "/horizDist", horizDist);
            Monologue.log(inp + "/vertDist", vertDist);
            Monologue.log(inp + "/deltaNoteVelo", deltaNoteVelo);

            Monologue.log(out + "/wristLocation", wristLocation);
            Monologue.log(out + "/linearWristRads", linearWristRads);
            Monologue.log(out + "/vertcialDistanceError", vertcialDistanceError);
            Monologue.log(out + "/newVerticalDist", newVerticalDist);
            Monologue.log(out + "/grav", grav);
        }

        return grav;
    }

    public static StemPosition iterativeSolveLowestPivotDelta(
        double startingPosition,
        Function<Double, StemPosition> positionGenerator
    ) {
        final double stepSize = Units.degreesToRadians(1.0);
        int tryCount = 0;
        double pivotRads = startingPosition;
        StemPosition lastStemPosition = positionGenerator.apply(pivotRads);
        while (!lastStemPosition.isValid() && tryCount < 90) {
            if (pivotRads > Math.PI / 4.0) {
                pivotRads -= stepSize;
            } else {
                pivotRads += stepSize;
            }
            lastStemPosition = positionGenerator.apply(pivotRads);
            tryCount++;
        }
        return lastStemPosition;
    }

    /**
     * There is no gurantee any of these fields will be used by the solver
     */
    public static record StemSolveInput(
        /**
         * This will define the current position of the stem
         * to be used in more optimized solves that desire as little
         * movement as possible.
         */
        StemPosition currentStemPosition,
        /**
         * This will define any static components of a solve like
         * pivot rads for a stationary pivot solve.
         */
        StemPosition desiredStemPosition,
        /**
         * The distance from the CENTER of the robot to the target
         */
        double horizDist,
        /**
         * The distance from the FLOOR to the target
         */
        double vertDist,
        /**
         * The average velocity of the note, this will be used in
         * gravity compensated solves.
         */
        double deltaNoteVelo
    ) {}

    public enum AimSolveStrategy {
        STATIONARY_WRIST(input -> {
            double wristRads = input.desiredStemPosition.wristRads;
            double telescopeMeters = input.desiredStemPosition.telescopeMeters;
            return StemPosition.fromRadians(
                linearSolvePivotTheta(
                    telescopeMeters,
                    wristRads,
                    input.horizDist(),
                    input.vertDist()
                ),
                wristRads,
                telescopeMeters
            );
        }),
        STATIONARY_PIVOT(input -> {
            double pivotRads = input.desiredStemPosition.pivotRads;
            double telescopeMeters = input.desiredStemPosition.telescopeMeters;
            return StemPosition.fromRadians(
                pivotRads,
                linearSolveWristTheta(
                    telescopeMeters,
                    pivotRads,
                    input.horizDist(),
                    input.vertDist(),
                    true
                ),
                telescopeMeters
            );
        }),
        STATIONARY_PIVOT_GRAVITY(input -> {
            double pivotRads = input.desiredStemPosition.pivotRads;
            double telescopeMeters = input.desiredStemPosition.telescopeMeters;
            return StemPosition.fromRadians(
                pivotRads,
                gravitySolveWristTheta(
                    telescopeMeters,
                    pivotRads,
                    input.horizDist(),
                    input.vertDist(),
                    input.deltaNoteVelo()
                ),
                telescopeMeters
            );
        }),
        STATIONARY_PIVOT_GRAVITY_TELESCOPE_EXTEND(input -> {
            double pivotRads = input.desiredStemPosition.pivotRads;
            double telescopeMeters = StemPosition.INTAKE.telescopeMeters;
            return StemPosition.fromRadians(
                pivotRads,
                gravitySolveWristTheta(
                    telescopeMeters,
                    pivotRads,
                    input.horizDist(),
                    input.vertDist(),
                    input.deltaNoteVelo()
                ),
                telescopeMeters
            );
        }),
        STATIONARY_PIVOT_TELESCOPE_EXTEND(input -> {
            double pivotRads = input.desiredStemPosition.pivotRads;
            double telescopeMeters = StemPosition.INTAKE.telescopeMeters;
            return StemPosition.fromRadians(
                pivotRads,
                linearSolveWristTheta(
                    telescopeMeters,
                    pivotRads,
                    input.horizDist(),
                    input.vertDist(),
                    true
                ),
                telescopeMeters
            );
        }),
        LEAST_MOVEMENT(input -> {
            return iterativeSolveLowestPivotDelta(
                input.currentStemPosition.pivotRads,
                pivotRads -> {
                    double telescopeMeters = Math.min(
                        input.currentStemPosition.telescopeMeters,
                        StemPosition.INTAKE.telescopeMeters
                    );
                    return StemPosition.fromRadians(
                        pivotRads,
                        gravitySolveWristTheta(
                            telescopeMeters,
                            pivotRads,
                            input.horizDist(),
                            input.vertDist(),
                            input.deltaNoteVelo()
                        ),
                        telescopeMeters
                    );
                }
            );
        });

        private final Function<StemSolveInput, StemPosition> solver;

        AimSolveStrategy(final Function<StemSolveInput, StemPosition> solver) {
            this.solver = solver;
        }

        public StemPosition solve(StemSolveInput input) {
            return solver.apply(input);
        }
    }
}
