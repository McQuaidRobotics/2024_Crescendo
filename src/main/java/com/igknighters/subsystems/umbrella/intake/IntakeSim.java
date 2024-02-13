package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeSim implements Intake {

    private final IntakeInputs inputs = new IntakeInputs();
    private final SimBoolean exit;

    public IntakeSim() {
        if (RobotBase.isReal()) {
            // In the event we use this to "disable" the intake on the real robot,
            // we don't want to crash the robot by trying to create a SimDevice
            exit = null;
            NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("ExitBeamBreak")
                    .getEntry("broken2")
                    .setBoolean(false);
        } else if (GlobalState.isUnitTest()) {
            // HAL requires unique allocations for each SimDevice,
            // in unit tests we don't care what this is actually called so just make it
            // random
            exit = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
        } else {
            exit = SimDevice.create("ExitBeamBreak").createBoolean("broken2", Direction.kInput, false);
        }

        BootupLogger.bootupLog("    Intake initialized (sim)");
    }

    @Override
    public void setVoltageOut(double volts) {
        setVoltageOut(volts, false);
    }

    @Override
    public boolean isExitBeamBroken() {
        return inputs.exitBeamBroken;
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (isExitBeamBroken() && !force) {
            volts = 0.0;
        }
        inputs.voltsLower = volts;
        inputs.voltsUpper = inputs.voltsLower * kIntake.UPPER_DIFF;
        inputs.radiansPerSecondLower = (volts / 12.0) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
        inputs.radiansPerSecondUpper = inputs.radiansPerSecondLower * kIntake.UPPER_DIFF;
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            var table = NetworkTableInstance.getDefault()
                    .getTable("SimDevices");
            inputs.exitBeamBroken = table.getSubTable("ExitBeamBreak")
                    .getEntry("broken2")
                    .getBoolean(false);
        } else {
            inputs.exitBeamBroken = exit.get();
        }

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
