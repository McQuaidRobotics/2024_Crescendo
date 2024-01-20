package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTarget extends TeleopSwerveBase {

    private Translation2d targetTranslation = FieldConstants.APRIL_TAG_FIELD
        .getTagPose(7)
        .get()
        .getTranslation()
        .toTranslation2d();

    public TeleopSwerveTarget(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
        targetTranslation = targetTranslation
            .plus(orientForUser(new Translation2d(
                        getRotationX() * 0.1,
                        getRotationY() * 0.1
                    )));

        var vt = orientForUser(new Translation2d(
                getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * 0.85,
                getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * 0.85));

        GlobalState.modifyField(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        var targetAngle = swerve.rotationRelativeToPose(
                Rotation2d.fromDegrees(180),
                targetTranslation.plus(new Translation2d(
                        vt.getX() * ConstValues.PERIODIC_TIME,
                        vt.getY() * ConstValues.PERIODIC_TIME)));
        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        Logger.recordOutput("/Swerve/rotvelo", rotVelo);

        if (Math.abs(vt.getX()) < 0.25 && Math.abs(vt.getX()) < 0.25 && Math.abs(rotVelo) < 0.75) {
            rotVelo = 0.0;
        }

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                rotVelo,
                swerve.getYawWrappedRot());

        swerve.driveChassisSpeeds(chassisSpeeds, false);
    }
}
