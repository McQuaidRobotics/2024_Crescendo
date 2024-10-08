package com.igknighters.commands;

import java.util.Set;
import java.util.function.Supplier;

import com.igknighters.commands.led.LedCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.led.LedAnimations;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Emotes {

    public static Command bopYourHead(AllSubsystems subsystems) {
        if (!subsystems.hasAllSubsystems()) return Commands.idle(subsystems.getEnabledLockFullSubsystemsArr());

        Supplier<Command> cmd = () -> {
            var swerve = SwerveCommands.driveChassisSpeed(
                subsystems.swerve.get(),
                new ChassisSpeeds(0.0, 0.0, 1.0)
            );
            var stem = Commands.repeatingSequence(
                StemCommands.moveTo(
                    subsystems.stem.get(),
                    StemPosition.fromDegrees(
                        60.0,
                        75.0,
                        kTelescope.MIN_METERS
                    ),
                    10.0
                ),
                StemCommands.moveTo(
                    subsystems.stem.get(),
                    StemPosition.fromDegrees(
                        70.0,
                        105.0,
                        kTelescope.MIN_METERS
                    ),
                    10.0
                )
            );
            var umbrella = UmbrellaCommands.stopShooter(subsystems.umbrella.get());
            var leds = Commands.repeatingSequence(
                LedCommands.animate(subsystems.led.get(), LedAnimations.SHOOTING),
                Commands.waitSeconds(0.5),
                LedCommands.animate(subsystems.led.get(), LedAnimations.Intake),
                Commands.waitSeconds(0.5)
            );

            return Commands.parallel(
                swerve,
                stem,
                umbrella,
                leds
            );
        };

        return Commands.defer(cmd, Set.of(subsystems.getEnabledLockFullSubsystemsArr()));
    }

    public static Command yes(AllSubsystems subsystems) {
        if (!subsystems.hasAllSubsystems()) return Commands.idle(subsystems.getEnabledLockFullSubsystemsArr());

        var umbrella = UmbrellaCommands.stopShooter(subsystems.umbrella.get());
        var swerve = Commands.idle(subsystems.swerve.get());
        var stem = Commands.sequence(
            StemCommands.moveTo(
                subsystems.stem.get(),
                StemPosition.fromDegrees(
                    90.0,
                    85.0,
                    kTelescope.MIN_METERS + 0.1
                ),
                3.0
            ),
            StemCommands.moveTo(
                subsystems.stem.get(),
                StemPosition.fromDegrees(
                    90.0,
                    113.0,
                    kTelescope.MIN_METERS + 0.1
                ),
                1.5
            ),
            StemCommands.moveTo(
                subsystems.stem.get(),
                StemPosition.fromDegrees(
                    90.0,
                    62.0,
                    kTelescope.MIN_METERS + 0.1
                ),
                1.5
            ),
            StemCommands.moveTo(
                subsystems.stem.get(),
                StemPosition.fromDegrees(
                    90.0,
                    90.0,
                    kTelescope.MIN_METERS + 0.1
                ),
                1.0
            )
        );
        var leds = LedCommands.animate(subsystems.led.get(), LedAnimations.TELEOP);

        return Commands.parallel(
            umbrella,
            swerve,
            stem,
            leds
        );
    }
}
