package com.igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            } else {
                swerve.setYaw(Rotation2d.fromDegrees(0));
            }
        });
    }
}
