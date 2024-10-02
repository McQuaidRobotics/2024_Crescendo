package com.igknighters.commands;

import com.igknighters.LED;
import com.igknighters.Localizer;
import com.igknighters.LED.LedAnimations;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetSpeakerCmd;
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
import com.igknighters.util.plumbing.DoubleMonad;

import edu.wpi.first.math.geometry.Translation2d;
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
                    .alongWith(UmbrellaCommands.idleShooter(umbrella, UmbrellaCommands::defaultIdleRPM))
                    .beforeStarting(() -> {
                        LED.sendAnimation(LedAnimations.INTAKE).withDuration(1.0);
                    })
            ).withName("Intake");
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            ControllerParent controller,
            Localizer localizer
    ) {
        return Commands.parallel(
                new TeleopSwerveTargetSpeakerCmd(swerve, controller, localizer),
                StemCommands.aimAtSpeaker(stem, false, localizer::pose, swerve::getChassisSpeed)).withName("Aim");
    }

    public static Command aimNotePass(
        Stem stem,
        Umbrella umbrella,
        Localizer localizer
    ) {
        return Commands.parallel(
            UmbrellaCommands.spinupShooter(umbrella, 4500, ShooterSpinupReason.ManualAimSpeaker),
            StemCommands.aimAtPassPoint(
                stem,
                new Translation2d(
                    1.0, 7.0
                ),
                2.5,
                false,
                localizer::pose
            )
        );
    }

    public static Command genericShoot(
            Swerve swerve,
            Stem stem,
            Umbrella umbrella,
            ControllerParent controller,
            Localizer localizer
    ) {
        Command cmd;
        String name;
        DoubleMonad targetRpm = DoubleMonad.of(umbrella::getShooterTargetSpeed)
            .scale(30.0 / Math.PI)
            .log("Aim/ShooterRPM");
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
                            controller,
                            localizer),
                    UmbrellaCommands.shoot(umbrella, targetRpm)
                ).until(() -> controller.leftTrigger(true).getAsDouble() < 0.5);
        } else {
            name = "Traditional Shoot";
            cmd = Commands.deadline(
                UmbrellaCommands.shoot(umbrella, targetRpm),
                StemCommands.holdAt(stem, stem.getStemPosition())
            );
        }

        return cmd.finallyDo(() -> {
            StemCommands.holdAt(
                    stem,
                    StemPosition.STOW
            ).schedule();
        }).withName(name);
    }
}
