package com.igknighters.subsystems.stem.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.igknighters.constants.ConstValues.kStem.kWrist;

public class WristSim implements Wrist  {
    private final WristInputs inputs;
    private final SingleJointedArmSim sim;
    private final PIDController pidController = new PIDController(
        kWrist.MOTOR_kP, kWrist.MOTOR_kI, kWrist.MOTOR_kD, 0.2
    );
    private Double setRadians = Units.degreesToRadians(55.0), AppliedVolts = 0.0;

    public WristSim() {
        sim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            1.0 / kWrist.MOTOR_TO_MECHANISM_RATIO, 
            0.07, //TODO: get real values
            0.3,
            kWrist.WRIST_MIN_ANGLE,
            kWrist.WRIST_MAX_ANGLE,
            false,
            kWrist.WRIST_MIN_ANGLE
        );
        sim.setState(setRadians, 0);
        inputs = new WristInputs(setRadians);
    }


    @Override
    public void setWristRadians(Double radians) {
        setRadians = radians;
        Double wristVoltageFeedback = pidController.calculate(
            sim.getAngleRads(), radians);
        sim.setInputVoltage(wristVoltageFeedback);
        AppliedVolts = wristVoltageFeedback;
    }

    @Override
    public Double getWristRadians() {
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

        Logger.processInputs("SuperStructure/Wrist", inputs);
    }


    
}
