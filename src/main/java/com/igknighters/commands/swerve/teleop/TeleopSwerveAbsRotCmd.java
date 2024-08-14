package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveAbsRotCmd extends TeleopSwerveBaseCmd {

    public TeleopSwerveAbsRotCmd(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
        Translation2d absRotation = orientForUser(new Translation2d(getRotationX(), getRotationY()));
        double rotVelo = swerve.rotVeloForRotation(absRotation.getAngle(), Units.degreesToRadians(0.75)) * absRotation.getNorm();

        Translation2d vt = orientForUser(
                new Translation2d(getTranslationX(), getTranslationY())).times(kSwerve.MAX_DRIVE_VELOCITY);

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                rotVelo,
                swerve.getYawWrappedRot());

        swerve.drive(chassisSpeeds, true);
    }

    public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
