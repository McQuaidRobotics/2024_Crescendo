package com.igknighters.commands.autos;

import com.igknighters.commands.autos.DynamicPath.DynPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicRoutines {
    public static SequentialCommandGroup TRANSLATE = new SequentialCommandGroup(
        DynPath.NOTE_CENTER.getCmd(),
        DynPath.AMP.getCmd(),
        DynPath.NOTE_LEFT.getCmd(),
        DynPath.SPEAKER.getCmd()
    );

    public static Command[] choosableDynamicRoutines() {
        Command[] choosableRoutines = new Command[]{
            TRANSLATE.withName("TRANSLATE")
        };
        return choosableRoutines;
    }
}
