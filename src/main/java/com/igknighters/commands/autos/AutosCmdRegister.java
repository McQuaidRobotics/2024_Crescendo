package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutosCmdRegister {
    public static class TestCmdMakers {
        public static InstantCommand makeAim(Double arg) {
            System.out.println("Aiming at " + arg);
            return new InstantCommand();
        }
    }

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
        } else {
            SpecializedNamedCommands.registerEmptyCommand(
                "Intake",
                Double.class
            );
        }
        if (allSubsystems.stem.isPresent()) {
            NamedCommands.registerCommand("StowPosition", new InstantCommand());
            NamedCommands.registerCommand("Aim", new InstantCommand());
            SpecializedNamedCommands.registerCommand(
                    "Aim",
                    SpecializedNamedCommand.fromMethod(
                            TestCmdMakers.class,
                            "makeAim",
                            Double.class));
        }

        SpecializedNamedCommands.generateSpecialized();
    }
}
