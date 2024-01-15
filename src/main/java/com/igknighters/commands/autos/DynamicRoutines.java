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

    public static SequentialCommandGroup TEST = new SequentialCommandGroup(
        new DynamicPath(5.5, 3.0, -60.0, 0.1).withName("path 1").getCmd(),
        new DynamicPath(3.55, 4.10, 180.0, 0.1).withName("path 2").getCmd(),
        new DynamicPath(5.5, 5.2, 60.0, 0.1).withName("path 3").getCmd(),
        new DynamicPath(5.5, 3.0, -60.0, 0.1).withName("path 4").getCmd(),
        new DynamicPath(3.55, 4.10, 180, 0.1).withName("path 5").getCmd(),
        new DynamicPath(5.5, 5.2, 60.0, 0.1).withName("path 6").getCmd(),
        new DynamicPath(6.5, 4.1, 180.0, 0.1).withName("path 7").getCmd()
    );

    public static Command[] choosableDynamicRoutines() {
        Command[] choosableRoutines = new Command[]{
            TEST.withName("TEST")
        };
        return choosableRoutines;
    }
}
