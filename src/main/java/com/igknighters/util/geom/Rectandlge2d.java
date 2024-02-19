package com.igknighters.util.geom;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Rectandlge2d {

    private final Translation2d topLeft, topRight, bottomLeft, bottomRight;

    public Rectandlge2d(
        Translation2d topLeft,
        Translation2d topRight,
        Translation2d bottomLeft,
        Translation2d bottomRight
    ) {
        this.topLeft = topLeft;
        this.bottomRight = bottomRight;
        this.topRight = topRight;
        this.bottomLeft = bottomLeft;
    }

    public Rectandlge2d(double x, double y, double width, double height) {
        this(
            new Translation2d(x, y),
            new Translation2d(x + width, y),
            new Translation2d(x, y + height),
            new Translation2d(x + width, y + height)
        );
    }

    public Rectandlge2d(Translation2d topLeft, Translation2d bottomRight) {
        this(
            topLeft,
            new Translation2d(bottomRight.getX(), topLeft.getY()),
            new Translation2d(topLeft.getX(), bottomRight.getY()),
            bottomRight
        );
    }

    public Translation2d getTopLeft() {
        return topLeft;
    }

    public Translation2d getBottomRight() {
        return bottomRight;
    }

    public Translation2d getBottomLeft() {
        return bottomLeft;
    }

    public Translation2d getTopRight() {
        return topRight;
    }

    public double getWidth() {
        return topLeft.getDistance(topRight);
    }

    public double getHeight() {
        return topLeft.getDistance(bottomLeft);
    }

    public boolean contains(Translation2d point) {
        return point.getX() >= topLeft.getX() && point.getX() <= topRight.getX() &&
            point.getY() >= topLeft.getY() && point.getY() <= bottomLeft.getY();
    }

    public boolean intersects(Rectandlge2d other) {
        return topLeft.getX() < other.getBottomRight().getX() &&
            topRight.getX() > other.getTopLeft().getX() &&
            topLeft.getY() < other.getBottomRight().getY() &&
            bottomLeft.getY() > other.getTopLeft().getY();
    }

    public Rectandlge2d translate(Translation2d translation) {
        return new Rectandlge2d(
            topLeft.plus(translation),
            topRight.plus(translation),
            bottomLeft.plus(translation),
            bottomRight.plus(translation)
        );
    }

    public Rectandlge2d scale(double scale) {
        return new Rectandlge2d(
            topLeft.times(scale),
            topRight.times(scale),
            bottomLeft.times(scale),
            bottomRight.times(scale)
        );
    }

    public Rectandlge2d rotateBy(Rotation2d rotation, Translation2d centerOfRotation) {
        // Rotate each corner around the center of rotation
        return new Rectandlge2d(
            topLeft.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            topRight.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            bottomLeft.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            bottomRight.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation)
        );
    }

    public Rectandlge2d mirror() {
        return new Rectandlge2d(
            new Translation2d(-topLeft.getX(), topLeft.getY()),
            new Translation2d(-topRight.getX(), topRight.getY()),
            new Translation2d(-bottomLeft.getX(), bottomLeft.getY()),
            new Translation2d(-bottomRight.getX(), bottomRight.getY())
        );
    }

}
