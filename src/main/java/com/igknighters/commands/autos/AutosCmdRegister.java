package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.AutoSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutosCmdRegister {
    public static void registerCommands(AllSubsystems allSubsystems) {
        if (!allSubsystems.hasAllSubsystems()) {
            return;
        }

        Umbrella umbrella = allSubsystems.umbrella.get();
        Stem stem = allSubsystems.stem.get();
        Swerve swerve = allSubsystems.swerve.get();

        SpecializedNamedCommands.registerCommand(
                "Intake",
                SpecializedNamedCommand.fromLambda(
                        (Object timeout) -> {
                            return HigherOrderCommands
                                    .intakeGamepiece(stem, umbrella)
                                    .withTimeout((Double) timeout);
                        },
                        Double.class))
                .withDefault(9999.0);

        NamedCommands.registerCommand(
                "Stow",
                StemCommands.holdAt(stem, StemPosition.STOW));

        SpecializedNamedCommands.registerCommand(
                "Spinup",
                SpecializedNamedCommand.fromLambda(
                        (Object rpm) -> {
                            return UmbrellaCommands
                                    .spinupShooter(umbrella, (Double) rpm);
                        },
                        Double.class))
                .withDefault(kControls.SHOOTER_RPM);

        NamedCommands.registerCommand(
                "Aim",
                StemCommands.aimAtSpeaker(stem, false));

        NamedCommands.registerCommand(
            "AutoShoot",
            Commands.parallel(
                new AutoSwerveTargetSpeaker(swerve),
                StemCommands.aimAtSpeaker(stem, true),
                UmbrellaCommands.waitUntilSpunUp(umbrella, kControls.SHOOTER_RPM, 1.2)
            ).andThen(
                UmbrellaCommands.shoot(umbrella)
            )
        );

        NamedCommands.registerCommand(
            "FeedShooter",
            UmbrellaCommands.shoot(umbrella)
        );
    }
}
