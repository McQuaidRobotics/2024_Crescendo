package com.igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.igknighters.commands.autos.SimplePathfindingCommand;
import com.igknighters.subsystems.swerve.Swerve;

public class SwerveCommands {
    public static Command commandXDrives(final Swerve swerve) {
        return swerve.runOnce(() -> {
            SwerveModuleState[] newModuleStates = {
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(270)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(180)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(90)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
            };
            swerve.setModuleStates(newModuleStates, false);
        }).andThen(new WaitCommand(0.2)).withName("commandXDrives");
    }

    public static Command commandStopDrives(final Swerve swerve) {
        return swerve.runOnce(() -> swerve.drive(new ChassisSpeeds(), false)).withName("commandStopDrives");
    }

    public static Command orientGyro(final Swerve swerve) {
        return swerve.runOnce(() -> {
            var alliance = DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Blue);
            if (alliance.equals(DriverStation.Alliance.Red)) {
                swerve.setYaw(Rotation2d.fromDegrees(180));
                var pose = new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(180));
                swerve.resetOdometry(pose);
            } else {
                swerve.setYaw(Rotation2d.fromDegrees(0));
                var pose = new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0));
                swerve.resetOdometry(pose);
            }
        });
    }

    public static Command driveToAmp(Swerve swerve) {
        return new SimplePathfindingCommand(
                new Pose2d(1.84, 7.2, Rotation2d.fromDegrees(90)),
                swerve)
                .flipForAlliance()
                .andThen(
                        new SimplePathfindingCommand(
                                new Pose2d(1.84, 7.5, Rotation2d.fromDegrees(90)),
                                swerve).flipForAlliance());
    }

    private static class PointTowardsCommand extends Command {
        private final Swerve swerve;
        private final Translation2d target;
        private double velo = 1.0;

        public PointTowardsCommand(Swerve swerve, Translation2d target) {
            this.swerve = swerve;
            this.target = target;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            velo = swerve.rotVeloForRotation(
                    swerve.rotationRelativeToPose(
                            new Rotation2d(), target));
            swerve.drive(
                    new ChassisSpeeds(0, 0, velo),
                    false);
        }

        @Override
        public boolean isFinished() {
            return velo < 0.05;
        }
    }

    public static Command pointTowards(Swerve swerve, Translation2d target) {
        return new PointTowardsCommand(swerve, target);
    }
}
