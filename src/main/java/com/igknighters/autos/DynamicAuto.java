package com.igknighters.autos;

import java.util.ArrayList;

import com.igknighters.autos.DynamicRoutines.DynPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicAuto implements DPBlock {
    private Command cmd;

    public DynamicAuto(DynPath... dynPaths) {
        ArrayList<Command> cmds = new ArrayList<Command>();
        for (DynPath dynPath : dynPaths) {
            cmds.add(dynPath.getCmd());
        }
        this.cmd = buildAuto(cmds);
    }
    public DynamicAuto(DPBlock... dynBlocks) {
        ArrayList<Command> cmds = new ArrayList<Command>();
        for (DPBlock dynBlock : dynBlocks) {
            cmds.add(dynBlock.getCmd());
        }
        this.cmd = buildAuto(cmds);
    }
    
    public Command buildAuto(ArrayList<Command> dynPathCmds) {
        return new SequentialCommandGroup(dynPathCmds.toArray(new Command[dynPathCmds.size()]));
    }

    public Command getCmd() {
        return cmd;
    }
}
