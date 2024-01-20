package com.igknighters.util;

import edu.wpi.first.math.geometry.Translation2d;

public class PointInPoly {
    public static boolean pointInPolyPrim(double x, double y, double[] polyX, double[] polyY) {
        int polyCorners = polyX.length;
        int i, j = polyCorners - 1;
        boolean oddNodes = false;

        for (i = 0; i < polyCorners; i++) {
            if ((polyY[i] < y && polyY[j] >= y) || (polyY[j] < y && polyY[i] >= y)) {
                if (polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x) {
                    oddNodes = !oddNodes;
                }
            }
            j = i;
        }

        return oddNodes;
    }

    public static boolean pointInPoly(Translation2d point, Translation2d[] poly) {
        double[] polyX = new double[poly.length];
        double[] polyY = new double[poly.length];

        for (int i = 0; i < poly.length; i++) {
            polyX[i] = poly[i].getX();
            polyY[i] = poly[i].getY();
        }

        return pointInPolyPrim(point.getX(), point.getY(), polyX, polyY);
    }
}
