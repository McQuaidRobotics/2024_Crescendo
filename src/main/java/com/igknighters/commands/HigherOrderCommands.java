package com.igknighters.commands;

import com.igknighters.LED;
import com.igknighters.LED.LedAnimations;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
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
                }).andThen(
                    UmbrellaCommands.intake(umbrella)
                        .until(() -> umbrella.holdingGamepiece()))
                )
            .andThen(
                StemCommands.holdAt(stem, StemPosition.STOW)
                    .alongWith(UmbrellaCommands.idleShooter(umbrella))
                    .beforeStarting(() -> {
                        LED.sendAnimation(LedAnimations.INTAKE).withDuration(1.0);
                    })
            ).withName("Intake");
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            ControllerParent controller) {
        return Commands.parallel(
                new TeleopSwerveTargetSpeaker(swerve, controller),
                StemCommands.aimAtSpeaker(stem, false)).withName("Aim");
    }

    public static Command genericShoot(
            Swerve swerve,
            Stem stem,
            Umbrella umbrella,
            ControllerParent controller) {
        Command cmd;
        String name;
        var reason = umbrella.popSpinupReason();
        if (reason.equals(ShooterSpinupReason.Amp)) {
            name = "Amp Shoot";
            cmd = Commands.sequence(
                    StemCommands.moveTo(stem, StemPosition.AMP_SCORE, 1.4),
                    umbrella.run(
                            () -> {
                                umbrella.spinupShooter(kControls.SHOOTER_RPM);
                                umbrella.runIntakeAt(-1.0, true);
                            }).withTimeout(0.3),
                    StemCommands.moveTo(stem, StemPosition.AMP_SAFE, 1.5));
        } else if (reason.equals(ShooterSpinupReason.AutoAimSpeaker)) {
            name = "Auto Aim Shoot";
            cmd = Commands.parallel(
                    HigherOrderCommands.aim(
                            swerve,
                            stem,
                            controller),
                    UmbrellaCommands.shoot(umbrella)
                ).until(() -> controller.leftTrigger(true).getAsDouble() < 0.5)
                .asProxy();
        } else {
            name = "Traditional Shoot";
            cmd = UmbrellaCommands.shoot(umbrella);
        }

        return cmd.finallyDo(
                umbrella::stopAll).andThen(
                        StemCommands.holdAt(
                                stem,
                                StemPosition.STOW)
                            .alongWith(UmbrellaCommands.idleShooter(umbrella))
                        )
                .withName(name);
    }
}
