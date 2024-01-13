package com.igknighters.autos;

import java.util.function.Supplier;

import com.igknighters.autos.DynamicRoutines.DynPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAuto implements DPBlock {
    private Supplier<Command> cmdSupplier;

    public DynamicAuto(DynPath... dynPaths) {
        this.cmdSupplier = () -> {
            Command[] pathCmds = new Command[dynPaths.length];
            for (int i = 0; i < dynPaths.length; i++) {
                pathCmds[i] = dynPaths[i].getCmd();
            }
            return new SequentialCommandGroup(pathCmds);
        };
    }
    public DynamicAuto(DPBlock... dynBlocks) {
        this.cmdSupplier = () -> {
            Command[] pathCmds = new Command[dynBlocks.length];
            for (int i = 0; i < dynBlocks.length; i++) {
                pathCmds[i] = dynBlocks[i].getCmd();
            }
            return new SequentialCommandGroup(pathCmds);
        };
    }

    public Command getCmd() {
        return cmdSupplier.get();
    }
}