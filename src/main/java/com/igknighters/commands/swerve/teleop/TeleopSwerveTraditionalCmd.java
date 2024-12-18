package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerBase;

public class TeleopSwerveTraditionalCmd extends TeleopSwerveBaseCmd {

    public TeleopSwerveTraditionalCmd(Swerve swerve, ControllerBase controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
        Translation2d vt = orientForUser(getTranslation())
                .times(kSwerve.MAX_DRIVE_VELOCITY);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                -getRotationX() * kSwerve.MAX_ANGULAR_VELOCITY, // invert because CCW is positive
                new Rotation2d(swerve.getYawRads())
        );

        swerve.drive(chassisSpeeds, true);
    }

    public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
