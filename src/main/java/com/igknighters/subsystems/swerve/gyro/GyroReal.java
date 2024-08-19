package com.igknighters.subsystems.swerve.gyro;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.HardwareIndex.SwerveHW;
import com.igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import com.igknighters.util.can.CANRetrier;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.util.logging.FaultManager;

import edu.wpi.first.math.util.Units;

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
    public void periodic() {
        FaultManager.captureFault(
                SwerveHW.Pigeon2,
                rollSignal, pitchSignal, yawSignal,
                rollVeloSignal, pitchVeloSignal, yawVeloSignal,
                xAccel, yAccel);

        super.pitchRads = Units.degreesToRadians(pitchSignal.getValueAsDouble());
        super.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValueAsDouble());
        super.rollRads = Units.degreesToRadians(rollSignal.getValueAsDouble());
        super.rollVelRadsPerSec = Units.degreesToRadians(rollVeloSignal.getValueAsDouble());
        super.yawRads = Units.degreesToRadians(yawSignal.getValueAsDouble());
        super.yawVelRadsPerSec = Units.degreesToRadians(yawVeloSignal.getValueAsDouble());

        log("XAccel", xAccel.getValueAsDouble());
        log("YAccel", yAccel.getValueAsDouble());
    }
}
