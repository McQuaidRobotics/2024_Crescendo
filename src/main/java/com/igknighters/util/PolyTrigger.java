package com.igknighters.util;

import java.util.function.Supplier;

import com.igknighters.GlobalState;
import com.igknighters.util.geom.Polygon2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PolyTrigger {
    private final Polygon2d polygon;
    private final Trigger trigger;

    public PolyTrigger(Supplier<Pose2d> poseSupplier, Polygon2d polygon) {
        this.polygon = polygon;
        this.trigger = new Trigger(() -> {
            Pose2d pose = poseSupplier.get();
            return polygon.contains(pose.getX(), pose.getY());
        });
    }

    public PolyTrigger(Supplier<Pose2d> poseSupplier, Translation2d... polygonVertecies) {
        this(poseSupplier, new Polygon2d(polygonVertecies));
    }

    public PolyTrigger(Polygon2d polygon) {
        this(GlobalState::getLocalizedPose, polygon);
    }

    public PolyTrigger(Translation2d... polygonVertecies) {
        this(new Polygon2d(polygonVertecies));
    }

    public Trigger toTrigger() {
        return trigger;
    }

    public Polygon2d getPolygon() {
        return polygon;
    }
}