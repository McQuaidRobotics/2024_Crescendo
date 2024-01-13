package com.igknighters.autos;

import java.util.function.Supplier;

import com.igknighters.RobotContainer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPath implements DPBlock {
    private Supplier<Command> cmdSupplier;
    private String name = "Dynamic Path Command";
    // private Pose2d current = 

    public DynamicPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
        cmdSupplier = () -> AutoBuilder.pathfindToPose(target, contraints, endVelo, rotDelay);
    }
    public DynamicPath(PathPlannerPath path, PathConstraints contraints, double rotDelay) {
        cmdSupplier = () -> AutoBuilder.pathfindThenFollowPath(path, contraints, rotDelay);
    }

    public DynamicPath withName(String name) {
        this.name = name;
        return this;
    }

    public Command getCmd() {
        return DynamicRoutines.loggedCommad(cmdSupplier.get().withName(name));
    }

    
}
