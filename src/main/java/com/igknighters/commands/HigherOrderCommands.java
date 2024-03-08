package com.igknighters.commands;

import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.controllers.ControllerParent;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HigherOrderCommands {

    public static Command intakeGamepiece(Stem stem, Umbrella umbrella) {
        return Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                Commands.idle().until(
                    () -> {
                        var pose = stem.getStemPosition();
                        return pose.wristRads > (StemPosition.INTAKE.wristRads
                                + kWrist.MIN_ANGLE) / 2.0
                                && pose.telescopeMeters > kTelescope.MIN_METERS;
                    })
                .andThen(
                    UmbrellaCommands.intake(umbrella)
                        .until(() -> umbrella.holdingGamepiece())),
                    StemCommands.holdAt(stem, StemPosition.STOW)
                )
                .withName("Intake");
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            ControllerParent controller) {
        return Commands.parallel(
                new TeleopSwerveTargetSpeaker(swerve, controller)
                        .withSpeedMultiplier(0.5),
                StemCommands.aimAtSpeaker(stem, false)).withName("Aim");
    }

    public static Command genericShoot(
            Swerve swerve,
            Stem stem,
            Umbrella umbrella,
            ControllerParent controller) {
        Command cmd;
        if (umbrella.popSpinupReason().equals(ShooterSpinupReason.Amp)) {
            cmd = Commands.sequence(
                    StemCommands.moveTo(stem, StemPosition.AMP_SCORE, 1.1),
                    umbrella.run(
                            () -> {
                                // Spinup while shooting to ensure the needed power is provided
                                umbrella.spinupShooterToRPM(6000);
                                umbrella.runIntakeAt(-1.0, true);
                            }).withTimeout(0.3),
                    StemCommands.moveTo(stem, StemPosition.AMP_SAFE, 1.5));
        } else if (umbrella.popSpinupReason().equals(ShooterSpinupReason.AutoAimSpeaker)) {
            cmd = Commands.parallel(
                    HigherOrderCommands.aim(
                            swerve,
                            stem,
                            controller),
                    UmbrellaCommands.shoot(
                            umbrella))
                    .until(() -> controller.leftTrigger(true).getAsDouble() < 0.5);
        } else {
            cmd = UmbrellaCommands.shoot(umbrella);
        }

        return cmd.finallyDo(
                umbrella::stopAll).andThen(
                        StemCommands.holdAt(
                                stem,
                                StemPosition.STOW))
                .withName("Shoot");
    }
}
