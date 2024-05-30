package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.TunableValues;
import com.igknighters.util.TunableValues.TunableDouble;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.GeomUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import com.igknighters.Localizer;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTargetSpeaker extends TeleopSwerveBase {

    private final Localizer localizer;
    private final TunableDouble lookaheadTime = TunableValues.getDouble("AutoAimLookaheadTime", 0.2);
    private final TunableDouble speedMult = TunableValues.getDouble("AutoAimSpeedMult", 0.4);

    public TeleopSwerveTargetSpeaker(Swerve swerve, ControllerParent controller, Localizer localizer) {
        super(swerve, controller);
        addRequirements(swerve);
        this.localizer = localizer;
    }

    @Override
    public void initialize() {
        swerve.resetRotController();
    }

    @Override
    public void execute() {
        Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
        Translation2d targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

        // GlobalState.modifyField2d(field -> {
        //     field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        // });

        Translation2d vt = orientForUser(new Translation2d(
                getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * speedMult.value(),
                getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * speedMult.value()));

        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                0.0,
                swerve.getYawWrappedRot());
        ChassisSpeeds currentChassisSpeeds = swerve.getChassisSpeed();

        ChassisSpeeds avgChassisSpeeds = new ChassisSpeeds(
                (desiredChassisSpeeds.vxMetersPerSecond + currentChassisSpeeds.vxMetersPerSecond) / 2.0,
                (desiredChassisSpeeds.vyMetersPerSecond + currentChassisSpeeds.vyMetersPerSecond) / 2.0,
                0.0);

        double distance = localizer.pose().getTranslation().getDistance(targetTranslation);

        double noteVelo = TunableValues.getDouble("Note Average Velo", kUmbrella.NOTE_VELO).value();

        Translation2d adjustedTarget = new Translation2d(
                targetTranslation.getX() - (avgChassisSpeeds.vxMetersPerSecond * (distance / noteVelo)),
                targetTranslation.getY() - (avgChassisSpeeds.vyMetersPerSecond * (distance / noteVelo)));

        // GlobalState.modifyField2d(field -> {
        //     field.getObject("adjustedTarget").setPose(new Pose2d(adjustedTarget, GeomUtil.ROTATION2D_ZERO));
        // });

        double lookaheadTimeValue = lookaheadTime.value();
        Translation2d lookaheadTranslation = localizer.pose().getTranslation()
            .minus(new Translation2d(
                avgChassisSpeeds.vxMetersPerSecond * lookaheadTimeValue,
                avgChassisSpeeds.vyMetersPerSecond * lookaheadTimeValue
            ));

        Rotation2d targetAngle = GeomUtil.rotationRelativeToPose(
                lookaheadTranslation,
                adjustedTarget
        ).plus(GeomUtil.ROTATION2D_PI);

        double rotVelo = swerve.rotVeloForRotation(targetAngle, Units.degreesToRadians(0.3));

        // GlobalState.modifyField2d(field -> {
        //     field.getObject("lookaheadRobot").setPose(new Pose2d(lookaheadTranslation, targetAngle));
        // });

        desiredChassisSpeeds.omegaRadiansPerSecond = rotVelo;

        swerve.drive(desiredChassisSpeeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        // GlobalState.modifyField2d(field -> {
        //     field.getObject("lookaheadRobot").setPoses();
        //     field.getObject("target").setPoses();
        //     field.getObject("adjustedTarget").setPoses();
        // });
    }
}
