package com.igknighters.autos;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPath implements DPBlock {
    private Supplier<Command> cmdSupplier;

    public DynamicPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
        cmdSupplier = () -> AutoBuilder.pathfindToPose(target, contraints, endVelo, rotDelay);
    }
    public DynamicPath(PathPlannerPath path, PathConstraints contraints, double rotDelay) {
        cmdSupplier = () -> AutoBuilder.pathfindThenFollowPath(path, contraints, rotDelay);
    }

    public Supplier<Command> getCmdSupplier() {
        return cmdSupplier;
    }
}
