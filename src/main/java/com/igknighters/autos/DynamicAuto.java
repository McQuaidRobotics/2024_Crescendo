package com.igknighters.autos;

import java.util.function.Supplier;

import com.igknighters.autos.DynamicRoutines.DynPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAuto implements DPBlock {
    private Command cmd;

    public DynamicAuto(DynPath... dynPaths) {
        Command[] pathCmds = new Command[dynPaths.length];
        for (int i = 0; i < dynPaths.length; i++) {
            pathCmds[i] = dynPaths[i].getCmd();
        }
        this.cmd = new SequentialCommandGroup(pathCmds).andThen(()-> new InstantCommand());
    }
    public DynamicAuto(DPBlock... dynBlocks) {
        Command[] pathCmds = new Command[dynBlocks.length];
        for (int i = 0; i < dynBlocks.length; i++) {
            pathCmds[i] = dynBlocks[i].getCmdSupplier().get();
        }
        this.cmd = new SequentialCommandGroup(pathCmds).andThen(()-> new InstantCommand());
    }

    public Supplier<Command> getCmdSupplier() {
        return () -> cmd;
    }
}