package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.Logger;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim implements Shooter {

    private final ShooterInputs inputs = new ShooterInputs();
    private final FlywheelSim flywheelSim = new FlywheelSim(
            DCMotor.getFalcon500(1),
            1.0,
            0.05);
    private final PIDController pid = new PIDController(
            kShooter.MOTOR_UPPER_kP * (Math.PI * 2),
            kShooter.MOTOR_UPPER_kI * (Math.PI * 2),
            kShooter.MOTOR_UPPER_kD * (Math.PI * 2),
            ConstValues.PERIODIC_TIME);

    public ShooterSim() {
    }

    @Override
    public double getSpeed() {
        return inputs.radiansPerSecondUpper;
    }

    @Override
    public double getTargetSpeed() {
        return inputs.targetRadiansPerSecondUpper;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecondUpper = speedRadPerSec;
        inputs.targetRadiansPerSecondLower = speedRadPerSec;

        var flywheelAppliedVolts = MathUtil.clamp(
                pid.calculate(flywheelSim.getAngularVelocityRadPerSec(), speedRadPerSec),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        flywheelSim.setInputVoltage(flywheelAppliedVolts);

        inputs.voltsUpper = flywheelAppliedVolts;
        inputs.voltsLower = flywheelAppliedVolts;
    }

    @Override
    public void setVoltageOut(double volts) {
        flywheelSim.setInputVoltage(volts);
        inputs.targetRadiansPerSecondUpper = 0.0;
        inputs.targetRadiansPerSecondLower = 0.0;
        inputs.voltsUpper = volts;
        inputs.voltsLower = volts;
    }

    @Override
    public void periodic() {
        flywheelSim.update(ConstValues.PERIODIC_TIME);

        inputs.ampsUpper = flywheelSim.getCurrentDrawAmps();
        inputs.ampsLower = flywheelSim.getCurrentDrawAmps();

        inputs.radiansPerSecondUpper = flywheelSim.getAngularVelocityRadPerSec();
        inputs.radiansPerSecondLower = flywheelSim.getAngularVelocityRadPerSec();

        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
