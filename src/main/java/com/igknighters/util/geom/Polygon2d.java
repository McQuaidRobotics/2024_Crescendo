package com.igknighters.util.geom;

import edu.wpi.first.math.geometry.Translation2d;

public class Polygon2d {

    private final Translation2d[] vertices;

    public Polygon2d(Translation2d... vertices) {
        this.vertices = vertices;
    }

    public Translation2d[] getVertices() {
        return vertices;
    }

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

    public boolean contains(double x, double y) {
        return contains(new Translation2d(x, y));
    }

    public boolean contains(Polygon2d polygon) {
        for (Translation2d vertex : polygon.getVertices()) {
            if (!contains(vertex)) {
                return false;
            }
        }
        return true;
    }

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
