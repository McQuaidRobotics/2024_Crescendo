package com.igknighters.commands.autos;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutosCmdRegister {
    public static void registerCommands(AllSubsystems allSubsystems) {
        NamedCommands.registerCommand("PickupPosition", new InstantCommand());
        NamedCommands.registerCommand("StowPosition", new InstantCommand());
        NamedCommands.registerCommand("Aim", new InstantCommand());
        NamedCommands.registerCommand("Shoot", new InstantCommand());
    }
}
