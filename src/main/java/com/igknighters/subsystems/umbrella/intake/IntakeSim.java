package com.igknighters.subsystems.umbrella.intake;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;

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
    }

    @Override
    public void turnIntake(double radians) {}

    @Override
    public void periodic() {
        inputs.hasGamePiece = GlobalState.Simulation.hasGamePiece();

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
