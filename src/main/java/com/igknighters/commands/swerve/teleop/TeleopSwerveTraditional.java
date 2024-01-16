package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTraditional extends TeleopSwerveBase {

    public TeleopSwerveTraditional(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
        Translation2d vt = adjustForSimOrientedControl(
            new Translation2d(getTranslationX(), getTranslationY())
        ).times(kSwerve.MAX_DRIVE_VELOCITY);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vt.getX(),
            vt.getY(),
            getRotationX() * kSwerve.MAX_ANGULAR_VELOCITY,
            swerve.getYawRot()
        );

        swerve.driveChassisSpeeds(chassisSpeeds, false, false);
    }
}
