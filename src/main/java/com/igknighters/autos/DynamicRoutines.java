package com.igknighters.autos;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.igknighters.constants.ConstValues.kAuto;

public class DynamicRoutines {
    public static Command loggedCommad(Command cmd) {
        return new FunctionalCommand(
            () -> {
                SmartDashboard.putString("DynamicCmd", cmd.getName());
                cmd.initialize();
            },
            cmd::initialize,
            (interupted) -> {
                SmartDashboard.putString("DynamicCmd", "");
                cmd.end(interupted);
            },
            cmd::isFinished,
            cmd.getRequirements().toArray(new Subsystem[0])
        );
    }

    static enum DynPath {
        SPEAKER(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        AMP(new DynamicPath(
                new Pose2d(new Translation2d(0.0, 5.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        SOURCE(new DynamicPath(
                new Pose2d(new Translation2d(6.0, 2.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(3.1, 4.1), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        SPEAKER_AMP_SOURCE(new DynamicAuto(
            DynPath.SPEAKER,
            DynPath.NOTE_CENTER,
            DynPath.SOURCE
        ));

        private DPBlock dynBlock;
        private DynPath(DynamicAuto dynAuto) {
            this.dynBlock = dynAuto;
        }
        private DynPath(DynamicPath dynPath) {
            this.dynBlock = dynPath.withName(this.name());
        }
        private DynPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
            this.dynBlock = new DynamicPath(target, contraints, endVelo, rotDelay).withName(this.name());
        }
        private DynPath(PathPlannerPath path, PathConstraints contraints, double rotDelay) {
            this.dynBlock = new DynamicPath(path, contraints, rotDelay).withName(this.name());
        }

        public Command getCmd() {
            return dynBlock.getCmd().withName(this.name());
        }
    }
}