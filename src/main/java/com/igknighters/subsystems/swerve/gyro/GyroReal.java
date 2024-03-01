package com.igknighters.subsystems.swerve.gyro;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.CANRetrier;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroReal implements Gyro {

    private final Pigeon2 gyro;
    private final StatusSignal<Double> rollSignal, pitchSignal, yawSignal;
    private final StatusSignal<Double> rollVeloSignal, pitchVeloSignal, yawVeloSignal;
    private final StatusSignal<Double> xAccel, yAccel;
    
    private final GyroInputs inputs = new GyroInputs();

    public GyroReal() {
        gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
        CANRetrier.retryStatusCode(() -> gyro.getConfigurator().apply(new Pigeon2Configuration()), 5);

        rollSignal = gyro.getRoll();
        pitchSignal = gyro.getPitch();
        yawSignal = gyro.getYaw();

        pitchVeloSignal = gyro.getAngularVelocityXDevice();
        rollVeloSignal = gyro.getAngularVelocityYDevice();
        yawVeloSignal = gyro.getAngularVelocityZDevice();

        xAccel = gyro.getAccelerationX();
        yAccel = gyro.getAccelerationY();

        rollSignal.setUpdateFrequency(100);
        pitchSignal.setUpdateFrequency(100);
        yawSignal.setUpdateFrequency(100);
        rollVeloSignal.setUpdateFrequency(100);
        pitchVeloSignal.setUpdateFrequency(100);
        yawVeloSignal.setUpdateFrequency(100);

        xAccel.setUpdateFrequency(100);
        yAccel.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();

        Supplier<Rotation3d> sup = () -> {
            return new Rotation3d(
                Math.toRadians(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, rollVeloSignal)),
                Math.toRadians(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, pitchVeloSignal)),
                Math.toRadians(yawSignal.getValue())
            );
        };
        GlobalState.setGyroRotSupplier(sup);

        BootupLogger.bootupLog("    Gyro initialized (real)");
    }

    @Override
    public double getPitchRads() {
        return inputs.pitchRads;
    }

    @Override
    public double getRollRads() {
        return inputs.rollRads;
    }

    @Override
    public double getYawRads() {
        return inputs.yawRads;
    }

    @Override
    public void setYawRads(double yawRads) {
        gyro.setYaw(Units.radiansToDegrees(yawRads));
    }

    @Override
    public void setVoltageOut(double volts) {
        DriverStation.reportError("\"GyroSim.setVoltageOut\" is not a supported operation", false);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                // pitchSignal, pitchVeloSignal,
                // rollSignal, rollVeloSignal,
                yawSignal /* , yawVeloSignal, */
                /* xAccel, yAccel */);

        inputs.pitchRads = Units.degreesToRadians(pitchSignal.getValue());
        inputs.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValue());
        inputs.rollRads = Units.degreesToRadians(rollSignal.getValue());
        inputs.yawRads = Units.degreesToRadians(yawSignal.getValue());
        inputs.yawVelRadsPerSec = Units.degreesToRadians(yawVeloSignal.getValue());

        Logger.recordOutput("Swerve/Gyro/XAccel", xAccel.getValue());
        Logger.recordOutput("Swerve/Gyro/YAccel", yAccel.getValue());

        Logger.processInputs("Swerve/Gyro", inputs);
    }
}
