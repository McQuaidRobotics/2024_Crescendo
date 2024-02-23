package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.AllianceFlip;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

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
        boolean blueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue);
        Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
        Translation2d targetTranslation = blueAlliance ? speaker : AllianceFlip.flipTranslation(speaker);

        var targetAngle = swerve.rotationRelativeToPose(
                Rotation2d.fromDegrees(180),
                targetTranslation);
        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        Logger.recordOutput("/Swerve/rotvelo", rotVelo);

        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(
                0.0,
                0.0,
                rotVelo);

        if (Math.abs(rotVelo) < 0.05) {
            isDone = true;
        }

        swerve.drive(desiredChassisSpeeds, false);
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
