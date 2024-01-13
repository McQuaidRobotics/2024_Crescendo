package com.igknighters.autos;

import java.util.ArrayList;

import com.igknighters.autos.DynamicRoutines.DynPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAuto implements DPBlock {
    private Command cmd;

    public DynamicAuto(DynPath... dynPaths) {
        Command[] pathCmds = new Command[dynPaths.length];
        for (int i = 0; i < dynPaths.length; i++) {
            CommandScheduler.getInstance().removeComposedCommand(dynPaths[i].getCmd());
            pathCmds[i] = dynPaths[i].getCmd();
        }
        this.cmd = new SequentialCommandGroup(pathCmds);
    }
    public DynamicAuto(DPBlock... dynBlocks) {
        Command[] pathCmds = new Command[dynBlocks.length];
        for (int i = 0; i < dynBlocks.length; i++) {
            CommandScheduler.getInstance().removeComposedCommand(dynBlocks[i].getCmd());
            pathCmds[i] = dynBlocks[i].getCmd();
        }
        this.cmd = new SequentialCommandGroup(pathCmds);
    }

    public Command getCmd() {
        return cmd;
    }
}