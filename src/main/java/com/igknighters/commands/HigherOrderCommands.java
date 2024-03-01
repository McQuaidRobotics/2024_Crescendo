package com.igknighters.commands;

import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.controllers.ControllerParent;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HigherOrderCommands {

    public static Command intakeGamepiece(Stem stem, Umbrella umbrella) {
        return Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                // Commands.idle().until(
                //         () -> {
                //             var pose = stem.getStemPosition();
                //             return pose.wristRads > (StemPosition.INTAKE.wristRads
                //                     + kWrist.MIN_ANGLE) / 2.0
                //                     && pose.telescopeMeters > kTelescope.MIN_METERS;
                //         }).andThen()
                UmbrellaCommands.intake(umbrella)
                .until(() -> umbrella.holdingGamepiece()))
                .andThen(StemCommands.moveTo(stem, StemPosition.STOW))
                .withName("Intake");
    }

    public static Command scoreAmp(Swerve swerve, Stem stem, Umbrella umbrella) {
        return Commands.parallel(
                StemCommands.moveTo(stem, StemPosition.AMP),
                SwerveCommands.driveToAmp(swerve),
                UmbrellaCommands.spinupShooter(umbrella, 1500, ShooterSpinupReason.Amp)
            ).withName("ScoreAmp");
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            ControllerParent controller) {
        return Commands.parallel(
                new TeleopSwerveTargetSpeaker(swerve, controller)
                        .withSpeedMultiplier(0.5),
                StemCommands.aimAtSpeaker(stem, false)
        ).withName("Aim");
    }

    public static Command genericShoot(
            Swerve swerve,
            Stem stem,
            Umbrella umbrella,
            ControllerParent controller) {
        Command cmd;
        if (umbrella.popSpinupReason().equals(ShooterSpinupReason.Amp)) {
            cmd = Commands.sequence(
                umbrella.run(
                    () -> {
                        umbrella.spinupShooter(umbrella.getShooterTargetSpeed());
                        umbrella.runIntakeAt(-1.0, true);
                    }
                ).withTimeout(0.9),
                Commands.parallel(
                    StemCommands.moveTo(
                        stem,
                        StemPosition.fromRadians(
                            StemPosition.AMP.pivotRads - Units.degreesToRadians(4.0),
                            StemPosition.AMP.wristRads,
                            StemPosition.AMP.telescopeMeters + Units.inchesToMeters(2.0)
                        )
                    ),
                    UmbrellaCommands.spinupShooter(
                        umbrella,
                        1000,
                        ShooterSpinupReason.Amp
                    )
                )
            );
        } else if (umbrella.popSpinupReason().equals(ShooterSpinupReason.AutoAimSpeaker)) {
            cmd = Commands.parallel(
                HigherOrderCommands.aim(
                    swerve,
                    stem,
                    controller
                ),
                UmbrellaCommands.shoot(
                    umbrella
                )
            ).until(() -> controller.leftTrigger(true).getAsDouble() < 0.5);
        } else {
            cmd = UmbrellaCommands.shoot(umbrella);
        }

        return cmd.finallyDo(
            umbrella::stopAll
        ).andThen(
            StemCommands.moveTo(
                stem,
                StemPosition.STOW
            )
        ).withName("Shoot");
    }
}
