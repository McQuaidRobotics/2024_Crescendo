package com.igknighters.commands;

import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.controllers.ControllerParent;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HigherOrderCommands {

    public static Command intakeGamepiece(Stem stem, Umbrella umbrella) {
        return Commands.race(
                StemCommands.holdAt(stem, StemPosition.fromDegrees(
                        11.0,
                        72.0,
                        kTelescope.MIN_METERS + Units.inchesToMeters(4.7))),
                UmbrellaCommands.intake(umbrella)).until(() -> umbrella.holdingGamepiece())
                .andThen(StemCommands.moveTo(stem, StemPosition.STOW));
    }

    public static Command scoreAmp(Swerve swerve, Stem stem, Umbrella umbrella) {
        return Commands.parallel(
                StemCommands.moveTo(stem, StemPosition.AMP),
                SwerveCommands.driveToAmp(swerve),
                UmbrellaCommands.spinupShooter(umbrella, 1500)).andThen(
                        UmbrellaCommands.shoot(umbrella));
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            Umbrella umbrella,
            ControllerParent controller) {
        return Commands.parallel(
                new TeleopSwerveTargetSpeaker(swerve, controller)
                        .withSpeedMultiplier(0.1),
                StemCommands.aimAtSpeaker(stem),
                UmbrellaCommands.spinupShooter(umbrella, kControls.SHOOTER_RPM));
    }
}
