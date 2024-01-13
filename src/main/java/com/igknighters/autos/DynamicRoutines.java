package com.igknighters.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicRoutines {
    public static SequentialCommandGroup TRANSLATE = new SequentialCommandGroup(
        new DynamicPath(2.0, 1.0, 0.0).withName("path 1").getCmd(),
        Commands.waitSeconds(3.0),
        new DynamicPath(-1.0, -2.0, 0.0).withName("path 2").getCmd()
    );

    public static Command[] choosableDynamicRoutines() {
        Command[] choosableRoutines = new Command[]{
            TRANSLATE.withName("TRANSLATE")
        };
        return choosableRoutines;
    }
}
