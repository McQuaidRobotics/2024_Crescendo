package com.igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.igknighters.Localizer;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.GeomUtil;

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
        return swerve.runOnce(() -> swerve.drive(new ChassisSpeeds(), true)).withName("commandStopDrives");
    }

    public static Command orientGyro(Swerve swerve, Localizer localizer) {
        return swerve.runOnce(() -> {
            if (AllianceFlip.isBlue()) {
                swerve.setYaw(GeomUtil.ROTATION2D_ZERO);
                var pose = new Pose2d(localizer.pose().getTranslation(), GeomUtil.ROTATION2D_ZERO);
                localizer.resetOdometry(pose, swerve.getModulePositions());
            } else {
                swerve.setYaw(GeomUtil.ROTATION2D_PI);
                var pose = new Pose2d(localizer.pose().getTranslation(), GeomUtil.ROTATION2D_PI);
                localizer.resetOdometry(pose, swerve.getModulePositions());
            }
        });
    }
}
