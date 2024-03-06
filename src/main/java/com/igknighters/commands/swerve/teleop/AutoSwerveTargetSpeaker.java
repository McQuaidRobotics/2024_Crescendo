package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.igknighters.constants.FieldConstants;

public class AutoSwerveTargetSpeaker extends Command {

    private final Swerve swerve;
    private boolean isDone = false;

    public AutoSwerveTargetSpeaker(Swerve swerve) {
        addRequirements(swerve);
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.resetRotController();
        isDone = false;
    }

    @Override
    public void execute() {
        Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
        Translation2d targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

        Rotation2d targetAngle = swerve.rotationRelativeToPose(
                Rotation2d.fromDegrees(180),
                targetTranslation);
        double rotVelo = swerve.rotVeloForRotation(targetAngle);

        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(
                0.0,
                0.0,
                rotVelo);

        if (Math.abs(rotVelo) < 0.01) {
            isDone = true;
        }

        swerve.drive(desiredChassisSpeeds, false);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
