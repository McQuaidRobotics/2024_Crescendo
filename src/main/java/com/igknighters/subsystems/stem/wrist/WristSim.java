package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.util.BootupLogger;

public class WristSim implements Wrist {
    private final WristInputs inputs;
    private final SingleJointedArmSim sim;
    private final PIDController pidController = new PIDController(
            kWrist.MOTOR_kP, kWrist.MOTOR_kI, kWrist.MOTOR_kD, 0.2);
    private double setRadians = Units.degreesToRadians(55.0), AppliedVolts = 0.0;

    public WristSim() {
        sim = new SingleJointedArmSim(
                DCMotor.getFalcon500(1),
                1.0,
                0.1, // TODO: get real values
                0.1,
                kWrist.WRIST_MIN_ANGLE,
                kWrist.WRIST_MAX_ANGLE,
                false,
                kWrist.WRIST_MIN_ANGLE);
        sim.setState(setRadians, 0);
        inputs = new WristInputs(setRadians);

        BootupLogger.bootupLog("    Wrist initialized (sim)");
    }

    @Override
    public void setWristRadians(Double radians) {
        setRadians = radians;
        double wristVoltageFeedback = pidController.calculate(
                sim.getAngleRads(), radians);
        sim.setInputVoltage(wristVoltageFeedback);
        AppliedVolts = wristVoltageFeedback;
    }

    @Override
    public double getWristRadians() {
        return inputs.radians;
    }

    @Override
    public void setVoltageOut(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            sim.setInputVoltage(0);
            AppliedVolts = 0.0;
        }

        sim.update(0.2);

        inputs.radians = Units.radiansToDegrees(sim.getAngleRads());
        inputs.radiansPerSecond = Units.radiansToDegrees(sim.getVelocityRadPerSec());
        inputs.volts = AppliedVolts;
        inputs.amps = sim.getCurrentDrawAmps();
        inputs.temp = 0.0;
        inputs.targetRadians = setRadians;

        Logger.processInputs("Stem/Wrist", inputs);
    }

}