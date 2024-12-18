package com.igknighters.util.geom;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A 2D polygon represented by a list of vertices.
 */
public class Polygon2d {

    private final Translation2d[] vertices;

    /**
     * Creates a new Polygon2d with the given vertices.
     * 
     * @param vertices The vertices of the polygon
     */
    public Polygon2d(Translation2d... vertices) {
        this.vertices = vertices;
    }

    /**
     * Gets the vertices of the polygon.
     * 
     * @return The vertices of the polygon
     */
    public Translation2d[] getVertices() {
        return vertices;
    }

    /**
     * Checks if the polygon contains the given point.
     * 
     * @param point The point to check
     * @return Whether the polygon contains the point
     */
    public boolean contains(Translation2d point) {
        int i, j;
        boolean c = false;
        for (i = 0, j = vertices.length - 1; i < vertices.length; j = i++) {
            if (((vertices[i].getY() > point.getY()) != (vertices[j].getY() > point.getY()))
                    && (point.getX() < (vertices[j].getX() - vertices[i].getX())
                            * (point.getY() - vertices[i].getY()) / (vertices[j].getY() - vertices[i].getY())
                            + vertices[i].getX())) {
                c = !c;
            }
        }
        return c;
    }

    /**
     * Checks if the polygon contains the given point.
     * 
     * @param x The x-coordinate of the point
     * @param y The y-coordinate of the point
     * @return Whether the polygon contains the point
     */
    public boolean contains(double x, double y) {
        return contains(new Translation2d(x, y));
    }

    /**
     * Checks if the polygon contains the given polygon.
     * 
     * @param polygon The polygon to check
     * @return Whether the polygon contains the given polygon
     */
    // TDOO: redo this to make it more robust
    public boolean contains(Polygon2d polygon) {
        for (Translation2d vertex : polygon.getVertices()) {
            if (!contains(vertex)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if the polygon intersects the given polygon.
     * 
     * @param polygon The polygon to check
     * @return Whether the polygon intersects the given polygon
     */
    // TDOO: redo this to make it more robust
    public boolean intersects(Polygon2d polygon) {
        for (Translation2d vertex : polygon.getVertices()) {
            if (contains(vertex)) {
                return true;
            }
        }
        for (Translation2d vertex : vertices) {
            if (polygon.contains(vertex)) {
                return true;
            }
        }
        return false;
    }
}
