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
            cmd::execute,
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
                new Pose2d(new Translation2d(1.28, 5.55), Rotation2d.fromDegrees(180)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        AMP(new DynamicPath(
                new Pose2d(new Translation2d(1.82, 7.72), Rotation2d.fromDegrees(90)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(5.78, 4.12), Rotation2d.fromDegrees(180)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(4.39, 4.86), Rotation2d.fromDegrees(-60)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        STAGE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(4.38, 3.33), Rotation2d.fromDegrees(60)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(2.9, 7.0), Rotation2d.fromDegrees(0.0)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0.0)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        NOTE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(2.9, 4.10), Rotation2d.fromDegrees(0.0)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                kAuto.DYN_END_VELO, 
                0.0)),
        SPEAKER_AMP_SOURCE(new DynamicAuto(
            DynPath.NOTE_RIGHT,
            DynPath.SPEAKER,
            DynPath.NOTE_CENTER,
            DynPath.NOTE_RIGHT
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