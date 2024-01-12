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
        0.05
    );
    private final PIDController pid = new PIDController(
        kShooter.kP * (Math.PI*2),
        kShooter.kI * (Math.PI*2),
        kShooter.kD * (Math.PI*2),
        ConstValues.PERIODIC_TIME
    );

    public ShooterSim() {}

    @Override
    public double getSpeed() {
        return inputs.radiansPerSecond;
    }

    @Override
    public void setSpeed(double speedRadPerSec) {
        inputs.targetRadiansPerSecond = speedRadPerSec;

        var flywheelAppliedVolts = MathUtil.clamp(
            pid.calculate(flywheelSim.getAngularVelocityRadPerSec(), speedRadPerSec),
            -RobotController.getBatteryVoltage(),
            RobotController.getBatteryVoltage()
        );

        flywheelSim.setInputVoltage(flywheelAppliedVolts);

        inputs.volts = flywheelAppliedVolts;
    }

    @Override
    public void setVoltageOut(double volts) {
        flywheelSim.setInputVoltage(volts);
        inputs.targetRadiansPerSecond = 0.0;
        inputs.volts = volts;
    }

    @Override
    public void periodic() {
        flywheelSim.update(ConstValues.PERIODIC_TIME);

        inputs.amps = flywheelSim.getCurrentDrawAmps();

        inputs.radiansPerSecond = flywheelSim.getAngularVelocityRadPerSec();

        Logger.processInputs("/Umbrella/Shooter", inputs);
    }
}
