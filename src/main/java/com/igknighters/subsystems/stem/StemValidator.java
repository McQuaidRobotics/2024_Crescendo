package com.igknighters.subsystems.stem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.igknighters.constants.ConstValues.kRobotGeometry;
import com.igknighters.subsystems.stem.StemVisualizer.StemVisualizerDot;
import com.igknighters.util.Channels.Sender;
import com.igknighters.util.geom.Rectangle2d;

public class StemValidator {
        /**
         * An effectively miniscule value used to define incredibly steep slopes
         */
        private static final double SMOL = 0.00000001;

        private static final Sender<StemVisualizerDot> DOT_SENDER = Sender.broadcast(
                        "StemVisualizerDots", StemVisualizerDot.class);

        public static class MechanismPoints {
                public final Translation2d wristAxelPoint, umbrellaBottomRightPoint, umbrellaTopRightPoint,
                                umbrellaBottomLeftPoint, umbrellaTopLeftPoint;

                public MechanismPoints(StemPosition stemPosition) {
                        // All points are from the perspective of the bottom floor of the umbrella
                        // parallel to the floor
                        // and the shooter wheels on the left.

                        Translation2d wristAxelPointTransform = new Translation2d(
                                        stemPosition.getTelescopeMeters() * Math.cos(stemPosition.getPivotRads()),
                                        stemPosition.getTelescopeMeters() * Math.sin(stemPosition.getPivotRads()));
                        this.wristAxelPoint = kRobotGeometry.PIVOT_LOCATION.plus(wristAxelPointTransform);

                        Translation2d umbrellaLengthVector = new Translation2d(
                                        kRobotGeometry.UMBRELLA_LENGTH
                                                        * Math.cos(Math.PI - (Math.PI - stemPosition.getWristRads())
                                                                        - stemPosition.getPivotRads()),
                                        kRobotGeometry.UMBRELLA_LENGTH * -1.0
                                                        * Math.sin(
                                                                        Math.PI - (Math.PI
                                                                                        - stemPosition.getWristRads())
                                                                                        - stemPosition.getPivotRads()));

                        this.umbrellaBottomLeftPoint = new Translation2d(
                                        kRobotGeometry.UMBRELLA_OFFSET
                                                        * Math.cos(stemPosition.getPivotRads() + (Math.PI / 2.0)
                                                                        - stemPosition.getWristRads()),
                                        kRobotGeometry.UMBRELLA_OFFSET
                                                        * Math.sin(stemPosition.getPivotRads() + (Math.PI / 2.0)
                                                                        - stemPosition.getWristRads()))
                                        .plus(this.wristAxelPoint);

                        this.umbrellaBottomRightPoint = umbrellaLengthVector.plus(this.umbrellaBottomLeftPoint);

                        Translation2d umbrellaTopTransform = new Translation2d(
                                        kRobotGeometry.UMBRELLA_HEIGHT
                                                        * Math.cos(stemPosition.getPivotRads() + (Math.PI / 2.0)
                                                                        - stemPosition.getWristRads()),
                                        kRobotGeometry.UMBRELLA_HEIGHT
                                                        * Math.sin(stemPosition.getPivotRads() + (Math.PI / 2.0)
                                                                        - stemPosition.getWristRads()));

                        this.umbrellaTopRightPoint = umbrellaBottomRightPoint.plus(umbrellaTopTransform);
                        this.umbrellaTopLeftPoint = umbrellaBottomLeftPoint.plus(umbrellaTopTransform);
                }

                private double getSmallest(double... values) {
                        double smallest = values[0];
                        for (double value : values)
                                if (value < smallest)
                                        smallest = value;
                        return smallest;
                }

                private double getLargest(double... values) {
                        double largest = values[0];
                        for (double value : values)
                                if (value > largest)
                                        largest = value;
                        return largest;
                }

                private Rectangle2d getCastedRect() {
                        double umbrellaRectLeft = getSmallest(umbrellaBottomLeftPoint.getX(),
                                        umbrellaBottomRightPoint.getX(),
                                        umbrellaTopLeftPoint.getX(), umbrellaTopRightPoint.getX());
                        double umbrellaRectRight = getLargest(umbrellaBottomLeftPoint.getX(),
                                        umbrellaBottomRightPoint.getX(),
                                        umbrellaTopLeftPoint.getX(), umbrellaTopRightPoint.getX());
                        double umbrellaRectTop = getLargest(umbrellaBottomLeftPoint.getY(),
                                        umbrellaBottomRightPoint.getY(),
                                        umbrellaTopLeftPoint.getY(), umbrellaTopRightPoint.getY());
                        double umbrellRectBottom = getSmallest(umbrellaBottomLeftPoint.getY(),
                                        umbrellaBottomRightPoint.getY(),
                                        umbrellaTopLeftPoint.getY(), umbrellaTopRightPoint.getY());
                        double width = Math.abs(umbrellaRectLeft - umbrellaRectRight);
                        double height = Math.abs(umbrellaRectTop - umbrellRectBottom);
                        return new Rectangle2d(umbrellaRectLeft, umbrellRectBottom, width, height);
                }

                public Translation2d[] getPoints() {
                        return new Translation2d[] {
                                        wristAxelPoint,
                                        umbrellaBottomRightPoint,
                                        umbrellaTopRightPoint,
                                        umbrellaBottomLeftPoint,
                                        umbrellaTopLeftPoint
                        };
                }

                private boolean outsideBounds() {
                        for (Translation2d point : getPoints()) {
                                if (!kRobotGeometry.BOUNDS.contains(point)) {
                                        return true;
                                }
                        }
                        return false;
                }
        }

        public enum ValidationResponse {
                VALID,
                COLLIDES_DRIVE_BASE,
                COLLIDES_BOUNDS,
                COLLIDES_DRIVE_BASE_AND_BOUNDS;

                /**
                 * Returns true if the response is valid
                 */
                public boolean isValid() {
                        return this.equals(VALID);
                }

                /**
                 * Resolves simple boolean expression to an enum variant,
                 * 
                 * @param drivebaseCollides If the stem/umbrella collides with the drivebase
                 * @param boundsCollide     If the umbrella is outside of the allowed bounds
                 */
                private static ValidationResponse collisionFrom(boolean drivebaseCollides, boolean boundsCollide) {
                        if (drivebaseCollides && !boundsCollide) {
                                return COLLIDES_DRIVE_BASE;
                        } else if (!drivebaseCollides && boundsCollide) {
                                return COLLIDES_BOUNDS;
                        } else if (drivebaseCollides && boundsCollide) {
                                return COLLIDES_DRIVE_BASE_AND_BOUNDS;
                        }
                        return VALID;
                }
        }

        /**
         * Returns the intersection of a line and a horizontal line at the given y
         * value.
         * 
         * @param pt1 The first point of the line
         * @param pt2 The second point of the line
         * @param y   The y value of the horizontal line
         * @return The x value of the intersection
         */
        private static double computeIntersection(Translation2d pt1, Translation2d pt2, double y) {
                double m = (pt2.getY() - pt1.getY()) / (pt2.getX() - pt1.getX());
                double b = (pt2.getY() - (m * pt2.getX())) / m;
                return (y - (m * b)) / m;
        }

        /**
         * Returns true if the passed in x value is inebtween the two planes that make
         * the verticle lines
         * of the side of the rectangle.
         */
        private static boolean withinXBoundsOf(Rectangle2d rect, double x) {
                return x > rect.getBottomLeft().getX() && x < rect.getBottomRight().getX();
        }

        private static double diffIfGreater(double greater, double lesser) {
                if (greater > lesser) {
                        return Math.abs(greater - lesser);
                }
                return 0.0;
        }

        /**
         * Returns an {@link InvalidationResponse} that defines if the position is valid
         * and
         * if it's not, how its invalid.
         */
        public static ValidationResponse validatePosition(StemPosition stemPosition) {

                // Gets the location of differect corners and joints on the stem mechanism and
                // umbrella within the x y coordinate plane from (0, 0)
                MechanismPoints mechPts = new MechanismPoints(stemPosition);

                DOT_SENDER.send(new StemVisualizerDot("wristAxel", mechPts.wristAxelPoint));
                DOT_SENDER.send(new StemVisualizerDot("umbrellaBottomRight", mechPts.umbrellaBottomRightPoint));
                DOT_SENDER.send(new StemVisualizerDot("umbrellaTopRight", mechPts.umbrellaTopRightPoint));
                DOT_SENDER.send(new StemVisualizerDot("umbrellaBottomLeft", mechPts.umbrellaBottomLeftPoint));
                DOT_SENDER.send(new StemVisualizerDot("umbrellaTopLeft", mechPts.umbrellaTopLeftPoint));

                // The x coordinates of where on the top and bottom of the drive base's
                // y coordinates intesects with the function of the line of the angle
                // of the wrist with the umbrella length from the umbrella point of
                // connection with the max tube and wrist axel.
                double wristDriveBaseTopIntersect = computeIntersection(
                                mechPts.umbrellaBottomLeftPoint,
                                new Translation2d(mechPts.umbrellaBottomRightPoint.getX() + SMOL,
                                                mechPts.umbrellaBottomRightPoint.getY() + SMOL),
                                kRobotGeometry.DRIVE_BASE.getTopY());

                double wristDriveBaseBottomIntersect = computeIntersection(
                                mechPts.umbrellaBottomLeftPoint,
                                new Translation2d(mechPts.umbrellaBottomRightPoint.getX() + SMOL,
                                                mechPts.umbrellaBottomRightPoint.getY() + SMOL),
                                kRobotGeometry.DRIVE_BASE.getBottomY());

                // The x coordinates of where on the top and bottom of the drive base's
                // y coordinates intesects with the function of the line of the angle
                // of the pivot with the stem length.
                double stemDriveBaseTopIntersect = computeIntersection(
                                new Translation2d(kRobotGeometry.PIVOT_LOCATION.getX() + SMOL,
                                                kRobotGeometry.PIVOT_LOCATION.getY() + SMOL),
                                mechPts.wristAxelPoint,
                                kRobotGeometry.DRIVE_BASE.getTopY());

                double stemDriveBaseBottomIntersect = computeIntersection(
                                new Translation2d(kRobotGeometry.PIVOT_LOCATION.getX() + SMOL,
                                                kRobotGeometry.PIVOT_LOCATION.getY() + SMOL),
                                mechPts.wristAxelPoint,
                                kRobotGeometry.DRIVE_BASE.getBottomY());

                DOT_SENDER.send(new StemVisualizerDot("wristDriveBaseTopIntersect",
                                new Translation2d(wristDriveBaseTopIntersect, kRobotGeometry.DRIVE_BASE.getTopY())));

                DOT_SENDER.send(new StemVisualizerDot("wristDriveBaseTopIntersect",
                                new Translation2d(wristDriveBaseTopIntersect, kRobotGeometry.DRIVE_BASE.getTopY())));

                DOT_SENDER.send(new StemVisualizerDot("wristDriveBaseTopIntersect",
                                new Translation2d(wristDriveBaseTopIntersect, kRobotGeometry.DRIVE_BASE.getTopY())));

                DOT_SENDER.send(new StemVisualizerDot("wristDriveBaseBottomIntersect",
                                new Translation2d(wristDriveBaseBottomIntersect,
                                                kRobotGeometry.DRIVE_BASE.getBottomY())));

                // Boolean logic for recognizing when the intersection points on the drive
                // base are within the interval of the drive base left and right side.
                // Also boolean logic to determine when the actual mechanism is outside the
                // drive base's allowed bounds.
                boolean topUmbrellaInterceptInRect = withinXBoundsOf(kRobotGeometry.DRIVE_BASE,
                                wristDriveBaseTopIntersect);
                boolean bottomUmbrellaIntrceptInRect = withinXBoundsOf(kRobotGeometry.DRIVE_BASE,
                                wristDriveBaseBottomIntersect);
                boolean isUmbrellaRightPointsBelowDriveBase = (mechPts.umbrellaBottomRightPoint
                                .getY() <= kRobotGeometry.DRIVE_BASE.getTopLeft().getY() ||
                                mechPts.umbrellaTopRightPoint.getY() <= kRobotGeometry.DRIVE_BASE.getTopLeft().getY());

                boolean topStemInterceptInRect = withinXBoundsOf(kRobotGeometry.DRIVE_BASE, stemDriveBaseTopIntersect);
                boolean bottomStemInterceptInRect = withinXBoundsOf(kRobotGeometry.DRIVE_BASE,
                                stemDriveBaseBottomIntersect);
                boolean isWristAxelPointBelowDrivebase = mechPts.wristAxelPoint.getY() <= kRobotGeometry.DRIVE_BASE
                                .getTopLeft()
                                .getY();

                SmartDashboard.putBoolean("topUmbrellaInterceptInRect", topUmbrellaInterceptInRect);
                SmartDashboard.putBoolean("bottomUmbrellaIntrceptInRect", bottomUmbrellaIntrceptInRect);
                SmartDashboard.putBoolean("isUmbrellaRightPointsBelowDriveBase", isUmbrellaRightPointsBelowDriveBase);
                SmartDashboard.putBoolean("topStemInterceptInRect", topStemInterceptInRect);
                SmartDashboard.putBoolean("bottomStemInterceptInRect", bottomStemInterceptInRect);
                SmartDashboard.putBoolean("isWristAxelPointBelowDrivebase", isWristAxelPointBelowDrivebase);

                boolean umbrellaCollidesDrivebase = isUmbrellaRightPointsBelowDriveBase
                                && (topUmbrellaInterceptInRect || bottomUmbrellaIntrceptInRect);
                boolean stemCollidesDrivebase = isWristAxelPointBelowDrivebase
                                && (topStemInterceptInRect || bottomStemInterceptInRect);
                boolean drivebaseCollides = umbrellaCollidesDrivebase || stemCollidesDrivebase;

                DOT_SENDER.send(new StemVisualizerDot("boundsTopRight", kRobotGeometry.BOUNDS.getTopRight()));
                DOT_SENDER.send(new StemVisualizerDot("boundsBottomRight", kRobotGeometry.BOUNDS.getBottomRight()));
                DOT_SENDER.send(new StemVisualizerDot("boundsTopLeft", kRobotGeometry.BOUNDS.getTopLeft()));
                DOT_SENDER.send(new StemVisualizerDot("boundsBottomLeft", kRobotGeometry.BOUNDS.getBottomLeft()));

                return ValidationResponse.collisionFrom(drivebaseCollides, mechPts.outsideBounds());
        }

        /**
         * Will
         */
        public static StemPosition stepTowardsTargetPosition(StemPosition currentState, StemPosition targetState,
                        double proportion) {
                SmartDashboard.putString("Current State Validation", validatePosition(currentState).name());

                ValidationResponse pivotMovementValidReason = validatePosition(StemPosition.fromRadians(
                                targetState.getPivotRads(),
                                currentState.getWristRads(),
                                currentState.getTelescopeMeters()));
                ValidationResponse wristMovementValidReason = validatePosition(StemPosition.fromRadians(
                                currentState.getPivotRads(),
                                targetState.getWristRads(),
                                currentState.getTelescopeMeters()));
                ValidationResponse telescopeMovementValidReason = validatePosition(StemPosition.fromRadians(
                                currentState.getPivotRads(),
                                currentState.getWristRads(),
                                targetState.getTelescopeMeters()));

                double midStatePivotRads = pivotMovementValidReason.isValid()
                                ? currentState.getPivotRads()
                                                + ((targetState.getPivotRads() - currentState.getPivotRads())
                                                                * proportion)
                                : currentState.getPivotRads();
                double midStateWristRads = wristMovementValidReason.isValid()
                                ? currentState.getWristRads()
                                                + ((targetState.getWristRads() - currentState.getWristRads())
                                                                * proportion)
                                : currentState.getWristRads();
                double midStateTelescopeMeters = telescopeMovementValidReason.isValid()
                                ? currentState.getTelescopeMeters() + ((targetState.getTelescopeMeters()
                                                - currentState.getTelescopeMeters()) * proportion)
                                : currentState.getTelescopeMeters();

                if (!pivotMovementValidReason.isValid()) {
                        StemPosition invalidPosition = StemPosition.fromRadians(
                                        targetState.getPivotRads(),
                                        currentState.getWristRads(),
                                        currentState.getTelescopeMeters());

                        MechanismPoints mechPts = new MechanismPoints(invalidPosition);
                        Rectangle2d umbrellaRect = mechPts.getCastedRect();

                        if (!pivotMovementValidReason.equals(ValidationResponse.COLLIDES_BOUNDS)) {
                                // Finds the lowest of the two right points on the umbrella imagining the
                                // umbrella at zero degrees from a side view in which it hangs off of the
                                // right side of the drive base. Then gets the distance below the top of
                                // the drive base and creates a mid state for pivot rads in which the
                                // new angle leads to the right points of the umbrella being at a y value
                                // that is equal to the invalid umbrella positions with the distance below
                                // the drive base subtracted from the umbrella's total y value

                                double lowestOfUmbrellaPoints = Math.min(
                                                mechPts.umbrellaBottomRightPoint.getY(),
                                                mechPts.umbrellaTopRightPoint.getY());
                                double difference = Math
                                                .abs(lowestOfUmbrellaPoints - kRobotGeometry.DRIVE_BASE.getTopY());
                                double opposite = (currentState.telescopeMeters
                                                * Math.sin(invalidPosition.getPivotRads())) + difference;
                                midStatePivotRads = Math.asin(opposite / currentState.getTelescopeMeters());

                        } else if (pivotMovementValidReason.equals(ValidationResponse.COLLIDES_BOUNDS)) {
                                // Casts the points of each corner on the umbrella into a rectangle. Then
                                // gets the difference outside the drive base's allowed bounds that rectangle is
                                // and adds or subtracts the differences so that rectangle is now within the
                                // drive base's allowed bounds. Then gets the distance from the pivot axel to
                                // the bottem left corner of the umbrella and turns it into a new mid state for
                                // the
                                // telescope. After this, the new telescope mid state is used to derive a new
                                // pivot rads mid state that when applies to the new telescope mid state results
                                // in the umbrella beingat the desired x and y coordinates from (0,0).

                                Rectangle2d bounds = kRobotGeometry.BOUNDS;

                                double rightDiff = diffIfGreater(umbrellaRect.getRightX(), bounds.getRightX());
                                double leftDiff = diffIfGreater(bounds.getLeftX(), umbrellaRect.getLeftX());
                                double topDiff = diffIfGreater(umbrellaRect.getTopY(), bounds.getTopY());
                                double bottomDiff = diffIfGreater(bounds.getBottomY(), umbrellaRect.getBottomY());

                                Translation2d midWristAxelPoint = mechPts.wristAxelPoint.plus(
                                                new Translation2d(
                                                                leftDiff - rightDiff,
                                                                bottomDiff - topDiff));

                                Translation2d pivotAxelToWristAxelSlope = new Translation2d(
                                                midWristAxelPoint.getX() - kRobotGeometry.PIVOT_LOCATION.getX(),
                                                midWristAxelPoint.getY() - kRobotGeometry.PIVOT_LOCATION.getY());

                                midStateTelescopeMeters = Math.sqrt(
                                                Math.pow(pivotAxelToWristAxelSlope.getX(), 2)
                                                                + Math.pow(pivotAxelToWristAxelSlope.getY(), 2));
                                midStatePivotRads = Math
                                                .asin(pivotAxelToWristAxelSlope.getY() / midStateTelescopeMeters);
                        }

                }

                SmartDashboard.putString("Target Stem Position", targetState.toString());
                SmartDashboard.putString("Current Stem Position", currentState.toString());
                SmartDashboard.putString("Mid State Stem Position", StemPosition
                                .fromRadians(midStatePivotRads, midStateWristRads, midStateTelescopeMeters).toString());

                return StemPosition.fromRadians(midStatePivotRads, midStateWristRads, midStateTelescopeMeters);
        }
}