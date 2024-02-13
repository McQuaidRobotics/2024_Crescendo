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

    private double speedMultiplier = 0.85;

    public TeleopSwerveTarget(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    public TeleopSwerveTarget withTarget(Translation2d target) {
        this.targetTranslation = target;
        return this;
    }

    public TeleopSwerveTarget withSpeedMultiplier(double multiplier) {
        this.speedMultiplier = multiplier;
        return this;
    }

    @Override
    public void execute() {
        Translation2d vt = orientForUser(
            new Translation2d(
                getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * speedMultiplier,
                getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * speedMultiplier
            ));

        GlobalState.modifyField(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        Rotation2d targetAngle = swerve.rotationRelativeToPose(
                Rotation2d.fromDegrees(180),
                targetTranslation.plus(new Translation2d(
                        vt.getX() * ConstValues.PERIODIC_TIME,
                        vt.getY() * ConstValues.PERIODIC_TIME)));

        Logger.recordOutput("/Commands/SwerveTarget/TargetAngle", targetAngle.getRadians());

        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        Logger.recordOutput("/Commands/SwerveTarget/RotationalVelocity", rotVelo);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                rotVelo,
                swerve.getYawWrappedRot());

        swerve.drive(chassisSpeeds, false);
    }
}
