package com.igknighters.autos;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.igknighters.constants.ConstValues.kAuto;

public class DynamicRoutines {
    static enum DynPath {
        SPEAKER(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                0.25, 
                0.0)),
        AMP(new DynamicPath(
                new Pose2d(new Translation2d(0.0, 5.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
                0.0)),
        SOURCE(new DynamicPath(
                new Pose2d(new Translation2d(6.0, 2.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                0.25, 
                0.0)),
        STAGE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
                0.0)),
        STAGE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
                0.0)),
        STAGE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
                0.0)),
        NOTE_LEFT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
                0.0)),
        NOTE_CENTER(new DynamicPath(
                new Pose2d(new Translation2d(3.1, 4.1), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                0.25, 
                0.0)),
        NOTE_RIGHT(new DynamicPath(
                new Pose2d(new Translation2d(3.0, 4.0), Rotation2d.fromDegrees(270)), 
                kAuto.DYNAMIC_PATH_CONSTRAINTS, 
                4.5, 
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
            this.dynBlock = dynPath;
        }
        private DynPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
            this.dynBlock = new DynamicPath(target, contraints, endVelo, rotDelay);
        }
        private DynPath(PathPlannerPath path, PathConstraints contraints, double rotDelay) {
            this.dynBlock = new DynamicPath(path, contraints, rotDelay);
        }

        public Command getCmd() {
            return dynBlock.getCmdSupplier().get().withName(this.name());
        }
    }
}