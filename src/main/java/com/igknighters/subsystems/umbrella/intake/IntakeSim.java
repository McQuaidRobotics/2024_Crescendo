package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeSim implements Intake {

    private final IntakeInputs inputs = new IntakeInputs();

    public IntakeSim() {}

    @Override
    public boolean hasGamePiece() {
        return inputs.hasGamePiece;
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.volts = volts;
        inputs.radiansPerSecond = (volts / 12.0) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
    }

    @Override
    public void turnIntake(double radians) {
        inputs.volts = 0.0;
        inputs.radiansPerSecond = 0.0;
    }

    @Override
    public void periodic() {
        inputs.hasGamePiece = GlobalState.hasGamePiece();

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
