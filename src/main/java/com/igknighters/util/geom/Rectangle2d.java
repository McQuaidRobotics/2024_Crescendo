package com.igknighters.util.geom;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// THIS CLASS IS NOT FULLY COMMENTED (womp womp)


public class Rectangle2d {

    private final Translation2d topLeft, topRight, bottomLeft, bottomRight;

    /**
     * Defines a rectangle that is drawn from its top left corner down to its bottom right corner
     * 
     * @param topLeft The top left point of the rectangle
     * @param topRight The top right point of the rectangle
     * @param bottomLeft The bottom left point of the rectangle
     * @param bottomRight The bottom right point of the rectangle
     */
    public Rectangle2d(
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

    /**
     * Defines a rectangle where (x,y) is the bottom left corner
     * 
     * @param x The x value for the top left corner of the rectangle
     * @param y The y value for the top left corner of the rectangle
     * @param width The width of the rectangle
     * @param height The height of the rectangle
     */
    public Rectangle2d(double x, double y, double width, double height) {
        this(
            new Translation2d(x, y + height),
            new Translation2d(x + width, y + height),
            new Translation2d(x, y),
            new Translation2d(x + width, y)
        );
    }

    public Rectangle2d(Translation2d center, double radius) {
        this(
            center.plus(new Translation2d(-radius, radius)),
            center.plus(new Translation2d(radius, radius)),
            center.plus(new Translation2d(-radius, -radius)),
            center.plus(new Translation2d(radius, -radius)
        ));
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

    public double getLeftX() {
        return Math.min(topLeft.getX(), bottomLeft.getX());
    }

    public double getRightX() {
        return Math.max(topRight.getX(), bottomRight.getX());
    }

    public double getTopY() {
        return Math.max(topLeft.getY(), topRight.getY());
    }

    public double getBottomY() {
        return Math.min(bottomLeft.getY(), bottomRight.getY());
    }

    public double getWidth() {
        return topLeft.getDistance(topRight);
    }

    public double getHeight() {
        return topLeft.getDistance(bottomLeft);
    }

    public boolean contains(Translation2d point) {
        return point.getX() >= getLeftX() && point.getX() <= getRightX() &&
            point.getY() <= getTopY() && point.getY() >= getBottomY();
    }

    public boolean intersects(Rectangle2d other) {
        return (
            contains(other.getTopLeft()) ||
            contains(other.getTopRight()) ||
            contains(other.getBottomLeft()) ||
            contains(other.getBottomRight())
        );
    }

    public Rectangle2d translate(Translation2d translation) {
        return new Rectangle2d(
            topLeft.plus(translation),
            topRight.plus(translation),
            bottomLeft.plus(translation),
            bottomRight.plus(translation)
        );
    }

    public Rectangle2d scale(double scale) {
        return new Rectangle2d(
            topLeft.times(scale),
            topRight.times(scale),
            bottomLeft.times(scale),
            bottomRight.times(scale)
        );
    }

    public Rectangle2d rotateBy(Rotation2d rotation, Translation2d centerOfRotation) {
        // Rotate each corner around the center of rotation
        return new Rectangle2d(
            topLeft.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            topRight.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            bottomLeft.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation),
            bottomRight.minus(centerOfRotation).rotateBy(rotation).plus(centerOfRotation)
        );
    }

    public Rectangle2d flipOverYAxis() {
        return new Rectangle2d(
            new Translation2d(-topLeft.getX(), topLeft.getY()),
            new Translation2d(-topRight.getX(), topRight.getY()),
            new Translation2d(-bottomLeft.getX(), bottomLeft.getY()),
            new Translation2d(-bottomRight.getX(), bottomRight.getY())
        );
    }

    public Rectangle2d flipOverXAxis() {
        return new Rectangle2d(
            new Translation2d(topLeft.getX(), -topLeft.getY()),
            new Translation2d(topRight.getX(), -topRight.getY()),
            new Translation2d(bottomLeft.getX(), -bottomLeft.getY()),
            new Translation2d(bottomRight.getX(), -bottomRight.getY())
        );
    }

    public Polygon2d asPolygon2d() {
        return new Polygon2d(topLeft, topRight, bottomRight, bottomLeft);
    }

    public Translation2d getCenter() {
        return topLeft.plus(bottomRight).times(0.5);
    }
}