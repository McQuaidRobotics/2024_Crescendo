package com.igknighters.subsystems.stem;

import edu.wpi.first.math.geometry.Translation2d;

import static com.igknighters.constants.ConstValues.kRobotGeometry;;

public class StemValidator {

    /**
     * Returns the intersection of a line and a horizontal line at the given y
     * value.
     * 
     * @param pt1 The first point of the line
     * @param pt2 The second point of the line
     * @param y   The y value of the horizontal line
     * @return The x value of the intersection
     */
    public static double computeIntersection(Translation2d pt1, Translation2d pt2, double y) {
        double m = (pt2.getY() - pt1.getY()) / (pt2.getX() - pt1.getX());
        double b = (pt2.getY() - (m * pt2.getX())) / m;
        return (y - (m * b)) / m;
    }

    public static boolean isValidPosition(StemPosition state) {
        // Point defining where the wrist axel is from (0,0)
        Translation2d wristAxelPoint = new Translation2d(
                (state.getTelescopeMeters() * Math.cos(state.getPivotRads())),
                state.getTelescopeMeters() * Math.sin(state.getPivotRads()));
        // Pivot axel point is constant that doesnt change
        wristAxelPoint = wristAxelPoint.plus(kRobotGeometry.PIVOT_LOCATION);

        Translation2d wristVector = new Translation2d(
                state.getTelescopeMeters() * Math.cos(state.getWristRads() - state.getPivotRads()),
                state.getTelescopeMeters() * -1.0 * Math.sin(state.getWristRads() - state.getPivotRads()));

        // Point defining where the umbrella bottom right corner is from (0,0)
        Translation2d wristEndPoint = wristAxelPoint.plus(wristVector);

        // Point deifning where the umbrella top right corner is from (0,0) | UNTESTED
        Translation2d wristTopRightVector = new Translation2d(
                kRobotGeometry.UMBRELLA_HEIGHT * Math.cos(state.getWristRads() - state.getPivotRads()),
                kRobotGeometry.UMBRELLA_HEIGHT * Math.sin(state.getWristRads() - state.getPivotRads()));
        wristTopRightVector = wristTopRightVector.plus(wristEndPoint);

        // Checks if the umbrella is impacting the floor
        if (wristEndPoint.getY() < 0 || wristTopRightVector.getY() < 0) {
            return false;
        }

        // The x coordinates of where on the top and bottom of the drive base's
        // y coordinates intesects with the function of the line of the angle
        // of the wrist with the wrist length from the wrist axel.
        double wristDriveBaseTopIntersect = computeIntersection(
                wristAxelPoint,
                wristEndPoint,
                kRobotGeometry.DRIVE_BASE.getTopLeft().getY());
        double wristDriveBaseBottomIntersect = computeIntersection(
                wristAxelPoint,
                wristEndPoint,
                kRobotGeometry.DRIVE_BASE.getBottomLeft().getY());

        // The x coordinates of where on the top and bottom of the drive base's
        // y coordinates intesects with the function of the line of the angle
        // of the pivot with the stem length.
        double telescopeDriveBaseTopIntersect = computeIntersection(
                kRobotGeometry.PIVOT_LOCATION,
                wristAxelPoint,
                kRobotGeometry.DRIVE_BASE.getTopLeft().getY());

        double telescopeDriveBaseBottomIntersect = computeIntersection(
                kRobotGeometry.PIVOT_LOCATION,
                wristAxelPoint,
                kRobotGeometry.DRIVE_BASE.getBottomLeft().getY());

        // Boolean logic for recognizing when the intersection points on the drive
        // base are within the interval of the drive base left and right side.
        // Also boolean logic to determine when the actual mechanism is below the
        // drive base's top coordinate.
        boolean isTopWristInRect = wristDriveBaseTopIntersect <= kRobotGeometry.DRIVE_BASE.getBottomRight().getX()
                && wristDriveBaseTopIntersect >= kRobotGeometry.DRIVE_BASE.getBottomLeft().getX();
        boolean isBottomWristInRect = wristDriveBaseBottomIntersect <= kRobotGeometry.DRIVE_BASE.getBottomRight().getX()
                && wristDriveBaseBottomIntersect >= kRobotGeometry.DRIVE_BASE.getBottomLeft().getX();
        boolean isWristEndPointBelowRectTop = wristEndPoint.getY() <= kRobotGeometry.DRIVE_BASE.getTopLeft().getY();

        boolean isTopStemInRect = telescopeDriveBaseTopIntersect <= kRobotGeometry.DRIVE_BASE.getBottomRight().getX()
                && telescopeDriveBaseTopIntersect >= kRobotGeometry.DRIVE_BASE.getBottomLeft().getX();
        boolean isBottomStemInRect = telescopeDriveBaseBottomIntersect <= kRobotGeometry.DRIVE_BASE.getBottomRight()
                .getX()
                && telescopeDriveBaseBottomIntersect >= kRobotGeometry.DRIVE_BASE.getBottomLeft().getX();
        boolean isWristAxelPointBelowRectTop = wristAxelPoint.getY() <= kRobotGeometry.DRIVE_BASE.getTopLeft().getY();

        boolean isUmbrellaValid = isWristEndPointBelowRectTop && (isTopWristInRect || isBottomWristInRect);
        boolean isStemValid = isWristAxelPointBelowRectTop && (isTopStemInRect || isBottomStemInRect);

        if (!isUmbrellaValid || !isStemValid) {
            return false;
        }

        return true;
    }

    public static StemPosition stepTowardsTargetPosition(StemPosition currentState, StemPosition targetState) {
        // Determine which mechanism can and can't move towards the target state at the
        // moment
        boolean pivotMovementValid = isValidPosition(StemPosition.fromRadians(
                targetState.getPivotRads(),
                currentState.getWristRads(),
                currentState.getTelescopeMeters()));
        boolean wristMovementValid = isValidPosition(StemPosition.fromRadians(
                currentState.getPivotRads(),
                targetState.getWristRads(),
                currentState.getTelescopeMeters()));
        boolean telescopeMovementValid = isValidPosition(StemPosition.fromRadians(
                currentState.getPivotRads(),
                currentState.getWristRads(),
                targetState.getTelescopeMeters()));

        // Generate new state that stunts mechanism that can't move from moving at the
        // moment
        double midStatePivotRads = pivotMovementValid ? targetState.getPivotRads() : currentState.getPivotRads();
        double midStateWristRads = wristMovementValid ? targetState.getWristRads() : currentState.getWristRads();
        double midStateTelescopeMeters = telescopeMovementValid ? targetState.getTelescopeMeters()
                : currentState.getTelescopeMeters();

        if (!pivotMovementValid) {
            StemPosition invalidState = StemPosition.fromRadians(
                    targetState.getPivotRads(),
                    currentState.getWristRads(),
                    currentState.getTelescopeMeters());

            // Point defining where the wrist axel is from (0,0)
            Translation2d wristAxelPoint = new Translation2d(
                    (invalidState.getTelescopeMeters() * Math.cos(invalidState.getPivotRads())),
                    invalidState.getTelescopeMeters() * Math.sin(invalidState.getPivotRads()));
            wristAxelPoint = wristAxelPoint.plus(kRobotGeometry.PIVOT_LOCATION);

            Translation2d wristVector = new Translation2d(
                    currentState.getTelescopeMeters()
                            * Math.cos(invalidState.getWristRads() - invalidState.getPivotRads()),
                    currentState.getTelescopeMeters() * -1.0
                            * Math.sin(invalidState.getWristRads() - invalidState.getPivotRads()));

            // Point defining where the wrist bottom right corner is from (0,0)
            Translation2d wristEndPoint = wristAxelPoint.plus(wristVector);

            // The distance from the drive base top y coordinate that the umbrella
            // is going past in the invalid state
            double intersectionDifference = Math.abs(
                    wristEndPoint.getY() - kRobotGeometry.DRIVE_BASE.getTopLeft().getY());

            // How much higher than the drive base the umbrella needs to be to
            // not be intersecting with the drive base
            double minimumDistFromDriveBaseTop = (currentState.getTelescopeMeters()
                    * Math.sin(invalidState.getPivotRads())) + intersectionDifference;

            // The new pivot angle that acheives the minimum distance from the drive
            // base top the umbrella needs to be in order to not intersect the drive base
            midStatePivotRads = Math.asin(
                    minimumDistFromDriveBaseTop / currentState.getTelescopeMeters());
        }

        return StemPosition.fromRadians(midStatePivotRads, midStateWristRads, midStateTelescopeMeters);
    }

}