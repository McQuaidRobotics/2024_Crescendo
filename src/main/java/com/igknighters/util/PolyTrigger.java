package com.igknighters.util;

import java.util.function.Supplier;

import com.igknighters.GlobalState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.awt.Polygon;
import java.awt.geom.PathIterator;

public class PolyTrigger {
    private final Polygon polygon;
    private final Trigger trigger;

    public PolyTrigger(Supplier<Pose2d> poseSupplier, Polygon polygon) {
        this.polygon = polygon;
        this.trigger = new Trigger(() -> {
            Pose2d pose = poseSupplier.get();
            return polygon.contains(pose.getX(), pose.getY());
        });
    }

    public PolyTrigger(Supplier<Pose2d> poseSupplier, Translation2d... polygon) {
        this(poseSupplier, PolyTrigger.getPolygonFromPoints(polygon));
    }

    public PolyTrigger(Polygon polygon) {
        this(GlobalState::getLocalizedPose, polygon);
    }

    public PolyTrigger(Translation2d... polygon) {
        this(PolyTrigger.getPolygonFromPoints(polygon));
    }

    public Trigger toTrigger() {
        return trigger;
    }

    public Polygon getPolygon() {
        return polygon;
    }

    public static Translation2d[] getPolygonPoints(Polygon polygon) {
        Translation2d[] points = new Translation2d[polygon.npoints];
        PathIterator pathIterator = polygon.getPathIterator(null);
        for (int i = 0; i < polygon.npoints; i++) {
            double[] pts = new double[6];
            pathIterator.currentSegment(pts);
            points[i] = new Translation2d(pts[0], pts[1]);
            pathIterator.next();
        }
        return points;
    }

    public static Polygon getPolygonFromPoints(Translation2d[] points) {
        int[] xPoints = new int[points.length];
        int[] yPoints = new int[points.length];
        for (int i = 0; i < points.length; i++) {
            xPoints[i] = (int) points[i].getX();
            yPoints[i] = (int) points[i].getY();
        }
        return new Polygon(xPoints, yPoints, points.length);
    }
}