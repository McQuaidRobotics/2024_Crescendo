package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

/** An example command that uses an example subsystem. */
public class TeleopSwerveAbsRot extends TeleopSwerveBase {

    public TeleopSwerveAbsRot(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
        Translation2d absRotation = new Translation2d(getRotationX(), getRotationY());
        double rotVelo = swerve.rotVeloForRotation(absRotation.getAngle()) * absRotation.getNorm();

        var translation = adjustForSimOrientedControl(new Translation2d(-getTranslationX(), -getTranslationY()));

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * kSwerve.MAX_DRIVE_VELOCITY,
            translation.getY() * kSwerve.MAX_DRIVE_VELOCITY,
            rotVelo,
            swerve.getYawRot()
        );

        swerve.driveChassisSpeeds(chassisSpeeds, true, false);
    }
}
