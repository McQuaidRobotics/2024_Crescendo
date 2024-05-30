package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.GeomUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.igknighters.constants.FieldConstants;

public class AutoSwerveTargetSpeaker extends Command {

    private final Swerve swerve;
    private final Supplier<Pose2d> poseSupplier;
    private boolean isDone = false;
    ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

    public AutoSwerveTargetSpeaker(Swerve swerve, Supplier<Pose2d> poseSupplier) {
        addRequirements(swerve);
        this.poseSupplier = poseSupplier;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.resetRotController();
        isDone = false;
        desiredChassisSpeeds.omegaRadiansPerSecond = 0.0;
    }

    @Override
    public void execute() {
        Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
        Translation2d targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

        // GlobalState.modifyField2d(field -> {
        //     field.getObject("targetTranslation").setPose(new Pose2d(targetTranslation, GeomUtil.ROTATION2D_ZERO));
        // });

        Rotation2d targetAngle = GeomUtil.rotationRelativeToPose(
                poseSupplier.get().getTranslation(),
                targetTranslation
        ).plus(GeomUtil.ROTATION2D_PI);
        double rotVelo = swerve.rotVeloForRotation(targetAngle, Units.degreesToRadians(1.5));

        desiredChassisSpeeds.omegaRadiansPerSecond = rotVelo;

        if (Math.abs(rotVelo) < 0.01
            && Math.abs(
                MathUtil.angleModulus(targetAngle.getRadians())
                - MathUtil.angleModulus(swerve.getYawRads())
            ) < Units.degreesToRadians(1.5)) {
            isDone = true;
        }

        swerve.drive(desiredChassisSpeeds, false);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(), false);
    }
}
