package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.util.BootupLogger;

public class WristSim implements Wrist {
    private final WristInputs inputs;
    private final FlywheelSim sim;
    private final PIDController pidController = new PIDController(
            kWrist.MOTOR_kP, kWrist.MOTOR_kI, kWrist.MOTOR_kD, ConstValues.PERIODIC_TIME);
    private double setRadians = Units.degreesToRadians(55.0), AppliedVolts = 0.0,
            motorRads = Wrist.mechanismRadsToMotorRads(setRadians);

    public WristSim() {
        // Use a flywheel to simulate a lead screw driving the wrist
        sim = new FlywheelSim(
                DCMotor.getFalcon500(1),
                1.0,
                0.2);
        inputs = new WristInputs(setRadians);

        BootupLogger.bootupLog("    Wrist initialized (sim)");
    }

    @Override
    public void setWristRadians(Double radians) {
        setRadians = MathUtil.clamp(radians, kWrist.MIN_ANGLE, kWrist.MAX_ANGLE);
        double desiredMotorRads = Wrist.mechanismRadsToMotorRads(setRadians);
        double volts = pidController.calculate(motorRads, desiredMotorRads);

        setVoltageOut(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public double getWristRadians() {
        return inputs.radians;
    }

    @Override
    public void setVoltageOut(double volts) {
        sim.setInputVoltage(volts);
        AppliedVolts = volts;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setVoltageOut(0.0);
        }

        sim.update(ConstValues.PERIODIC_TIME);

        motorRads = sim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;

        //TODO: Fix motorRadsToMechanismRads function

        inputs.radiansPerSecond = (inputs.radians - setRadians)
                / ConstValues.PERIODIC_TIME;
        inputs.radians = setRadians;
        inputs.volts = AppliedVolts;
        inputs.amps = sim.getCurrentDrawAmps();
        inputs.temp = 0.0;
        inputs.targetRadians = setRadians;

        Logger.processInputs("Stem/Wrist", inputs);
    }

}
