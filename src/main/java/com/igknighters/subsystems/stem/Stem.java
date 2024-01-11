package com.igknighters.subsystems.stem;

import com.igknighters.subsystems.stem.pivot.*;
import com.igknighters.subsystems.stem.telescope.*;
import com.igknighters.subsystems.stem.wrist.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stem extends SubsystemBase {

    private final Pivot pivot;
    private final Telescope telescope;
    private final Wrist wrist;

    public Stem() {
        if (RobotBase.isSimulation()) {
            pivot = new PivotSim();
            telescope = new TelescopeSim();
            wrist = new WristSim();
        } else {
            pivot = new PivotReal();
            telescope = new TelescopeReal();
            wrist = new WristReal();
        }
    }

    @Override
    public void periodic() {
        pivot.periodic();
        telescope.periodic();
        wrist.periodic();
    }
}
