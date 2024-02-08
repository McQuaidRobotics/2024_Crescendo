package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutosCmdRegister {
    static class TestCmdMakers {
        public static InstantCommand makeAim(Double arg) {
            System.out.println("Aiming at " + arg);
            return new InstantCommand();
        }
    }

    public static void registerCommands(AllSubsystems allSubsystems) {
        NamedCommands.registerCommand("PickupPosition", new InstantCommand());
        NamedCommands.registerCommand("StowPosition", new InstantCommand());
        NamedCommands.registerCommand("Aim", new InstantCommand());
        NamedCommands.registerCommand("Shoot", new InstantCommand());
        try {
            SpecializedNamedCommands.registerCommand(
                "Aim",
                SpecializedNamedCommand.fromMethod(
                    TestCmdMakers.class.getMethod("makeAim", Double.class),
                    Double.class
            ));
        } catch (NoSuchMethodException e) {
            e.printStackTrace();
        }

        SpecializedNamedCommands.generateSpecialized();
    }
}
