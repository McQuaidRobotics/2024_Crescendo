package com.igknighters.subsystems.swerve.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.HardwareIndex.SwerveHW;
import com.igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.FaultManager;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class GyroReal extends Gyro {

    private final Pigeon2 gyro;
    private final StatusSignal<Double> rollSignal, pitchSignal, yawSignal;
    private final StatusSignal<Double> rollVeloSignal, pitchVeloSignal, yawVeloSignal;
    private final StatusSignal<Double> xAccel, yAccel;

    public GyroReal(RealSwerveOdometryThread odoThread) {
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

        CANSignalManager.registerSignals(
                kSwerve.CANBUS,
                rollSignal, pitchSignal,
                rollVeloSignal, pitchVeloSignal,
                xAccel, yAccel);

        odoThread.addGyroStatusSignals(yawSignal, yawVeloSignal);

        gyro.optimizeBusUtilization(1.0);

        BootupLogger.bootupLog("    Gyro initialized (real)");
    }

    @Override
    public double getPitchRads() {
        return super.pitchRads;
    }

    @Override
    public double getRollRads() {
        return super.rollRads;
    }

    @Override
    public double getYawRads() {
        return super.yawRads;
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
        FaultManager.captureFault(
                SwerveHW.Pigeon2,
                rollSignal, pitchSignal, yawSignal,
                rollVeloSignal, pitchVeloSignal, yawVeloSignal,
                xAccel, yAccel);

        super.pitchRads = Units.degreesToRadians(pitchSignal.getValue());
        super.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValue());
        super.rollRads = Units.degreesToRadians(rollSignal.getValue());
        super.rollVelRadsPerSec = Units.degreesToRadians(rollVeloSignal.getValue());
        super.yawRads = Units.degreesToRadians(yawSignal.getValue());
        super.yawVelRadsPerSec = Units.degreesToRadians(yawVeloSignal.getValue());

        log("XAccel", xAccel.getValue());
        log("YAccel", yAccel.getValue());
    }
}
