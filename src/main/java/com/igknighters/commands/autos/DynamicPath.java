package com.igknighters.commands.autos;

import java.util.function.Function;

import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.subsystems.swerve.Swerve;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPath {

    static enum DynPath {
        SPEAKER(new DynamicPath(FieldPositions.SPEAKER)),
        AMP(new DynamicPath(FieldPositions.AMP)),
        STAGE_CENTER(new DynamicPath(FieldPositions.STAGE_CENTER)),
        STAGE_LEFT(new DynamicPath(FieldPositions.STAGE_LEFT)),
        STAGE_RIGHT(new DynamicPath(FieldPositions.STAGE_RIGHT)),
        NOTE_LEFT(new DynamicPath(FieldPositions.NOTE_LEFT)),
        NOTE_CENTER(new DynamicPath(FieldPositions.NOTE_CENTER)),
        NOTE_RIGHT(new DynamicPath(FieldPositions.NOTE_RIGHT));

        private DynamicPath dynBlock;
        private DynPath(DynamicPath dynPath) {
            this.dynBlock = dynPath.withName(this.name());
        }

        public Command getCmd(Swerve swerve) {
            return dynBlock.getCmd(swerve);
        }
    }

    private Function<Swerve, Command> cmdResolver;
    private String name = "Dynamic Path Command";

    public DynamicPath(double x, double y, double rotationDegrees, double rotationDelay) {
        var targetPose = new Pose2d(
            new Translation2d(x, y), 
            Rotation2d.fromDegrees(rotationDegrees)
        );

        cmdResolver = (swerve) -> new SimplePathfindingCommand(
            targetPose,
            0.0,
            rotationDelay,
            kAuto.DYNAMIC_PATH_CONSTRAINTS,
            swerve
        );
    }
    public DynamicPath(double x, double y, double rotationDegrees) {
        var targetPose = new Pose2d(
            new Translation2d(x, y), 
            Rotation2d.fromDegrees(rotationDegrees)
        );

        cmdResolver = (swerve) -> new SimplePathfindingCommand(
            targetPose,
            swerve
        );
    }
    public DynamicPath(Pose2d targetPose) {
        cmdResolver = (swerve) -> new SimplePathfindingCommand(
            targetPose,
            swerve
        );
    }
    public DynamicPath(Pose2d target, PathConstraints contraints, double endVelo, double rotDelay) {
        cmdResolver = (swerve) -> new SimplePathfindingCommand(
            target,
            endVelo,
            rotDelay,
            contraints,
            swerve
        );
    }

    public DynamicPath withName(String name) {
        this.name = name;
        return this;
    }

    public Command getCmd(Swerve swerve) {
        return cmdResolver.apply(swerve).withName(name);
    }
}
