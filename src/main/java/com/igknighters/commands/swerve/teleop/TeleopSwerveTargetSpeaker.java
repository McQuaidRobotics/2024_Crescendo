package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.AllianceFlip;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTargetSpeaker extends TeleopSwerveBase {

    private final Translation2d targetTranslation;

    public TeleopSwerveTargetSpeaker(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
        boolean blueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue);
        var speaker = FieldConstants.Speaker.CENTER_SPEAKER_OPENING.getTranslation();
        targetTranslation = blueAlliance ? speaker : AllianceFlip.flipTranslation(speaker);
    }

    @Override
    public void initialize() {
        swerve.resetRotController();
    }

    @Override
    public void execute() {
        GlobalState.modifyField2d(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        Translation2d vt = orientForUser(new Translation2d(
                getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * 0.4,
                getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * 0.4));

        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                0.0,
                swerve.getYawWrappedRot());
        ChassisSpeeds currentChassisSpeeds = GlobalState.getFieldRelativeVelocity();

        ChassisSpeeds avgChassisSpeeds = new ChassisSpeeds(
                (desiredChassisSpeeds.vxMetersPerSecond + currentChassisSpeeds.vxMetersPerSecond) / 2.0,
                (desiredChassisSpeeds.vyMetersPerSecond + currentChassisSpeeds.vyMetersPerSecond) / 2.0,
                (desiredChassisSpeeds.omegaRadiansPerSecond + currentChassisSpeeds.omegaRadiansPerSecond) / 2.0);

        double distance = GlobalState.getLocalizedPose().getTranslation().getDistance(targetTranslation);

        Translation2d adjustedTarget = new Translation2d(
                targetTranslation.getX() - (avgChassisSpeeds.vxMetersPerSecond * (distance / kUmbrella.NOTE_VELO)),
                targetTranslation.getY() - (avgChassisSpeeds.vyMetersPerSecond * (distance / kUmbrella.NOTE_VELO)));

        GlobalState.modifyField2d(field -> {
            field.getObject("adjustedTarget").setPose(new Pose2d(adjustedTarget, new Rotation2d()));
        });

        var targetAngle = swerve.rotationRelativeToPose(
                Rotation2d.fromDegrees(180),
                adjustedTarget);
        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        Logger.recordOutput("/Swerve/rotvelo", rotVelo);

        desiredChassisSpeeds.omegaRadiansPerSecond = rotVelo;

        swerve.drive(desiredChassisSpeeds, false);
    }
}
