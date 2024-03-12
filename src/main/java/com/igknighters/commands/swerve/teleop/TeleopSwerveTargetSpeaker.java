package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.TunableValues;
import com.igknighters.util.geom.AllianceFlip;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.igknighters.GlobalState;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTargetSpeaker extends TeleopSwerveBase {

    private final double lookaheadTimeDefault = 0.2;
    private double speedMult = 0.4;

    public TeleopSwerveTargetSpeaker(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
        addRequirements(swerve);
    }

    public TeleopSwerveTargetSpeaker withSpeedMultiplier(double speedMult) {
        this.speedMult = speedMult;
        return this;
    }

    @Override
    public void initialize() {
        swerve.resetRotController();
    }

    @Override
    public void execute() {
        Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
        Translation2d targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

        GlobalState.modifyField2d(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        Translation2d vt = orientForUser(new Translation2d(
                getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * speedMult,
                getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * speedMult));

        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                0.0,
                swerve.getYawWrappedRot());
        ChassisSpeeds currentChassisSpeeds = GlobalState.getFieldRelativeVelocity();

        ChassisSpeeds avgChassisSpeeds = new ChassisSpeeds(
                (desiredChassisSpeeds.vxMetersPerSecond + currentChassisSpeeds.vxMetersPerSecond) / 2.0,
                (desiredChassisSpeeds.vyMetersPerSecond + currentChassisSpeeds.vyMetersPerSecond) / 2.0,
                0.0);

        double distance = GlobalState.getLocalizedPose().getTranslation().getDistance(targetTranslation);

        Translation2d adjustedTarget = new Translation2d(
                targetTranslation.getX() - (avgChassisSpeeds.vxMetersPerSecond * (distance / kUmbrella.NOTE_VELO)),
                targetTranslation.getY() - (avgChassisSpeeds.vyMetersPerSecond * (distance / kUmbrella.NOTE_VELO)));

        GlobalState.modifyField2d(field -> {
            field.getObject("adjustedTarget").setPose(new Pose2d(adjustedTarget, new Rotation2d()));
        });

        double lookaheadTime = TunableValues.getDouble("AutoAimLookaheadTime", lookaheadTimeDefault).get();

        Translation2d lookaheadTranslation = swerve.getPose().getTranslation()
                .minus(
                        new Translation2d(
                                avgChassisSpeeds.vxMetersPerSecond * lookaheadTime,
                                avgChassisSpeeds.vyMetersPerSecond * lookaheadTime));

        Rotation2d targetAngle = swerve.rotationRelativeToPose(
                lookaheadTranslation,
                Rotation2d.fromDegrees(180),
                adjustedTarget);
        double rotVelo = swerve.rotVeloForRotation(targetAngle);

        GlobalState.modifyField2d(field -> {
            field.getObject("lookaheadRobot").setPose(new Pose2d(lookaheadTranslation, targetAngle));
        });

        desiredChassisSpeeds.omegaRadiansPerSecond = rotVelo;

        swerve.drive(desiredChassisSpeeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        GlobalState.modifyField2d(field -> {
            field.getObject("lookaheadRobot").setPoses();
            field.getObject("target").setPoses();
            field.getObject("adjustedTarget").setPoses();
        });
    }
}
