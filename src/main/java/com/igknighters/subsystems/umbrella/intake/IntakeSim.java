package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeSim implements Intake {

    private final IntakeInputs inputs = new IntakeInputs();
    private final SimBoolean entrance, exit;

    public IntakeSim() {
        if (GlobalState.isUnitTest()) {
            // HAL requires unique allocations for each SimDevice,
            // in unit tests we don't care what this is actually called so just make it
            // random
            entrance = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
            exit = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
        } else {
            entrance = SimDevice.create("EntranceBeamBreak").createBoolean("broken", Direction.kInput, false);
            exit = SimDevice.create("ExitBeamBreak").createBoolean("broken", Direction.kInput, false);
        }
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.volts = volts;
        inputs.radiansPerSecond = (volts / 12.0) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
    }

    @Override
    public void turnIntakeRads(double radians) {
        inputs.volts = 0.0;
        inputs.radiansPerSecond = 0.0;
    }

    @Override
    public boolean isEntranceBeamBroken() {
        return inputs.entranceBeamBroken;
    }

    @Override
    public boolean isExitBeamBroken() {
        return inputs.exitBeamBroken;
    }

    @Override
    public void periodic() {
        inputs.entranceBeamBroken = entrance.get();
        inputs.exitBeamBroken = exit.get();

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
