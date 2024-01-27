package com.igknighters.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.awt.Polygon;
import java.awt.geom.PathIterator;

public class PolyTrigger {
    private Supplier<Pose2d> poseSupplier;
    private Polygon polygon;
    private Trigger trigger;

    public PolyTrigger(Supplier<Pose2d> poseSupplier, Polygon polygon) {
        this.poseSupplier = poseSupplier;
        this.polygon = polygon;
        trigger = new Trigger(() -> polygon.contains(poseSupplier.get().getX(), poseSupplier.get().getY()));
    }

    public PolyTrigger onTrue(Command cmd) {
        trigger.onTrue(cmd);
        return this;
    }

    public PolyTrigger onFalse(Command cmd) {
        trigger.onFalse(cmd);
        return this;
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
}