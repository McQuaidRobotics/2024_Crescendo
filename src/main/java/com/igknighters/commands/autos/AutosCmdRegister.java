package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.FieldConstants;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.pathplanner.lib.auto.NamedCommands;

public class AutosCmdRegister {
    public static void registerCommands(AllSubsystems allSubsystems) {
        if (allSubsystems.umbrella.isPresent() && allSubsystems.stem.isPresent()) {
            Umbrella umbrella = allSubsystems.umbrella.get();
            Stem stem = allSubsystems.stem.get();

            SpecializedNamedCommands.registerCommand(
                "Intake",
                SpecializedNamedCommand.fromLambda(
                        (Object timeout) -> {
                            return HigherOrderCommands
                                    .intakeGamepiece(stem, umbrella)
                                    .withTimeout((Double) timeout);
                        },
                        Double.class)
            ).withDefault(9999.0);

            NamedCommands.registerCommand(
                "Stow",
                StemCommands.holdAt(stem, StemPosition.STOW)
            );

            SpecializedNamedCommands.registerCommand(
                "Spinup",
                SpecializedNamedCommand.fromLambda(
                        (Object rpm) -> {
                            return UmbrellaCommands
                                .spinupShooter(umbrella, (Double) rpm);
                        },
                        Double.class)
            ).withDefault(3780.0);

            SpecializedNamedCommands.registerCommand(
                "Spinup",
                SpecializedNamedCommand.fromLambda(
                        (Object rpm) -> {
                            return UmbrellaCommands
                                .spinupShooter(umbrella, (Double) rpm);
                        },
                        Double.class)
            ).withDefault(3780.0);

            NamedCommands.registerCommand(
                "Aim",
                StemCommands.aimAt(
                    stem,
                    FieldConstants.Speaker.SPEAKER_CENTER,
                    3780.0
                )
            );

        } else if (allSubsystems.umbrella.isPresent()) {
            Umbrella umbrella = allSubsystems.umbrella.get();

            SpecializedNamedCommands.registerEmptyCommand(
                "Intake",
                Double.class
            );

            SpecializedNamedCommands.registerCommand(
                "Spinup",
                SpecializedNamedCommand.fromLambda(
                        (Object rpm) -> {
                            return UmbrellaCommands
                                .spinupShooter(umbrella, (Double) rpm);
                        },
                        Double.class)
            ).withDefault(3780.0);
        } else if (allSubsystems.stem.isPresent()) {
            Stem stem = allSubsystems.stem.get();

            SpecializedNamedCommands.registerEmptyCommand(
                "Intake",
                Double.class
            );

            NamedCommands.registerCommand(
                "Stow",
                StemCommands.holdAt(stem, StemPosition.STOW)
            );
        }

        SpecializedNamedCommands.generateSpecialized();
    }
}
