package com.igknighters.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPath implements DPBlock {
    private Command cmd;

    public DynamicPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
        cmd = AutoBuilder.pathfindToPose(target, contraints, endVelo, rotDelay);
    }
    public DynamicPath(PathPlannerPath path, PathConstraints contraints, double rotDelay) {
        cmd = AutoBuilder.pathfindThenFollowPath(path, contraints, rotDelay);
    }
    
    // public DynamicPath withName(String name) {
    //     cmd = cmd.withName(name);
    //     return this;
    // }

    public Command getCmd() {
        return cmd; //TODO crashes if I use .withName for some reason
    }

}
