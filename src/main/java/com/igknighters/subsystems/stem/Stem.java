package com.igknighters.subsystems.stem;

import com.igknighters.subsystems.stem.elevator.*;
import com.igknighters.subsystems.stem.pivot.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stem extends SubsystemBase {

    private final Elevator elevator;
    private final Pivot pivot;

    public Stem() {
        if (RobotBase.isSimulation()) {
            elevator = new ElevatorSim();
            pivot = new PivotSim();
        } else {
            elevator = new ElevatorReal();
            pivot = new PivotReal();
        }
    }

    @Override
    public void periodic() {
        elevator.periodic();
        pivot.periodic();
    }
}
