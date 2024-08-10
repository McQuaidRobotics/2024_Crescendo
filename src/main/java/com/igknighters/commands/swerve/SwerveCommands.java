package com.igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.igknighters.commands.autos.SimplePathfindingCommand;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;

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
            if (AllianceFlip.isBlue()) {
                swerve.setYaw(Rotation2d.fromDegrees(0));
                var pose = new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0));
                swerve.resetOdometry(pose);
            } else {
                swerve.setYaw(Rotation2d.fromDegrees(180));
                var pose = new Pose2d(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(180));
                swerve.resetOdometry(pose);
            }
        });
    }

    public static Command driveToAmp(Swerve swerve) {
        return new SimplePathfindingCommand(
                new Pose2d(1.84, 7.2, Rotation2d.fromDegrees(-90)),
                swerve)
                .flipForAlliance()
                .withEndVelo(1.0)
                .withConstraints(0.5)
                .andThen(
                        new SimplePathfindingCommand(
                                new Pose2d(1.84, 7.5, Rotation2d.fromDegrees(-90)),
                                swerve).flipForAlliance().withConstraints(0.2));
    }

    private static abstract class PointTowardsCommand extends Command {
        private final Swerve swerve;
        private ChassisSpeeds velo = new ChassisSpeeds();

        public PointTowardsCommand(Swerve swerve) {
            this.swerve = swerve;
            addRequirements(swerve);
        }

        abstract Rotation2d getTarget();

        @Override
        public void execute() {
            velo.omegaRadiansPerSecond = swerve.rotVeloForRotation(getTarget());
            swerve.drive(velo, false);
        }

        @Override
        public boolean isFinished() {
            return velo.omegaRadiansPerSecond < 0.05;
        }

        @Override
        public String getName() {
            return "PointTowards";
        }
    }

    public static Command pointTowards(Swerve swerve, Translation2d target) {
        return new PointTowardsCommand(swerve) {
            @Override
            Rotation2d getTarget() {
                return swerve.rotationRelativeToPose(
                        new Rotation2d(), target);
            }
        };
    }

    public static Command pointTowards(Swerve swerve, Rotation2d target) {
        return new PointTowardsCommand(swerve) {
            @Override
            Rotation2d getTarget() {
                return target;
            }
        };
    }

    public static Command driveChassisSpeed(Swerve swerve, final ChassisSpeeds speeds) {
        return Commands.run(
            () -> swerve.drive(speeds, false),
            swerve
        );
    }
}
