package com.igknighters.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.igknighters.autos.DynamicPath.DynPath;

public class DynamicRoutines {
    public static SequentialCommandGroup NOTE_SPEAKER_NOTE_AMP = new SequentialCommandGroup(
        DynPath.NOTE_LEFT.getCmd(),
        DynPath.SPEAKER.getCmd(),
        DynPath.NOTE_CENTER.getCmd(),
        DynPath.AMP.getCmd(),
        new DynamicPath(3.0, 4.0, 270).getCmd()
    );
}
