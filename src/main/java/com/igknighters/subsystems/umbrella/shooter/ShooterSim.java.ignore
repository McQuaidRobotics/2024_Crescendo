package com.igknighters.subsystems.umbrella.shooter;

import org.littletonrobotics.junction.Logger;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.util.BootupLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
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
        BootupLogger.bootupLog("    Shooter initialized (sim)");
    }

    @Override
    public double getSpeed() {
        return inputs.radiansPerSecondRight;
    }

    @Override
    public double getTargetSpeed() {
        return inputs.targetRadiansPerSecondRight;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecondRight = speedRadPerSec;
        inputs.targetRadiansPerSecondLeft = speedRadPerSec;

        var flywheelAppliedVolts = MathUtil.clamp(
                pid.calculate(flywheelSim.getAngularVelocityRadPerSec(), speedRadPerSec),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        flywheelSim.setInputVoltage(flywheelAppliedVolts);

        inputs.voltsRight = flywheelAppliedVolts;
        inputs.voltsLeft = flywheelAppliedVolts;
    }

    @Override
    public void setVoltageOut(double volts) {
        flywheelSim.setInputVoltage(volts);
        inputs.targetRadiansPerSecondRight = 0.0;
        inputs.targetRadiansPerSecondLeft = 0.0;
        inputs.voltsRight = volts;
        inputs.voltsLeft = volts;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) setVoltageOut(0.0);

        flywheelSim.update(ConstValues.PERIODIC_TIME);

        inputs.ampsRight = flywheelSim.getCurrentDrawAmps();
        inputs.ampsLeft = flywheelSim.getCurrentDrawAmps();

        inputs.radiansPerSecondRight = flywheelSim.getAngularVelocityRadPerSec();
        inputs.radiansPerSecondLeft = flywheelSim.getAngularVelocityRadPerSec();

        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
