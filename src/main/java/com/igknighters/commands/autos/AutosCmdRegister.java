package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
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
            NamedCommands.registerCommand("Shoot", new InstantCommand());
            NamedCommands.registerCommand("PickupPosition", new InstantCommand());
        }
        if (allSubsystems.stem.isPresent()) {
            NamedCommands.registerCommand("StowPosition", new InstantCommand());
            NamedCommands.registerCommand("Aim", new InstantCommand());
            SpecializedNamedCommands.registerCommand(
            "Aim",
                SpecializedNamedCommand.fromMethod(
                    TestCmdMakers.class,
                    "makeAim",
                    Double.class
            ));
        }

        SpecializedNamedCommands.generateSpecialized();
    }
}
