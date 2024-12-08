package igknighters.subsystems.swerve.gyro;

import java.util.function.BiConsumer;

import sham.ShamSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import igknighters.SimCtx;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.HardwareIndex.SwerveHW;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.can.CANRetrier;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.BootupLogger;
import igknighters.util.logging.FaultManager;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import monologue.Annotations.IgnoreLogged;

public class GyroReal extends Gyro {

    private final Pigeon2 gyro;
    private final BaseStatusSignal rollSignal, pitchSignal;
    private final BaseStatusSignal rollVeloSignal, pitchVeloSignal;

    @IgnoreLogged
    private final RealSwerveOdometryThread odoThread;

    public GyroReal(RealSwerveOdometryThread odoThread, SimCtx simCtx) {
        this.odoThread = odoThread;

        gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
        CANRetrier.retryStatusCode(() -> gyro.getConfigurator().apply(new Pigeon2Configuration()), 5);

        rollSignal = gyro.getRoll();
        pitchSignal = gyro.getPitch();

        pitchVeloSignal = gyro.getAngularVelocityXWorld();
        rollVeloSignal = gyro.getAngularVelocityYWorld();

        CANSignalManager.registerSignals(
                kSwerve.CANBUS,
                rollSignal, pitchSignal,
                rollVeloSignal, pitchVeloSignal);

        odoThread.addGyroStatusSignals(
                gyro.getYaw(),
                gyro.getAngularVelocityZWorld(),
                gyro.getAccelerationX(),
                gyro.getAccelerationY());

        gyro.optimizeBusUtilization(0.0, 1.0);

        if (simCtx.isActive()) {
            BiConsumer<Time, AngularVelocity> simapplier = (time, rate) -> {
                Pigeon2SimState state = gyro.getSimState();
                state.addYaw(rate.times(time));
                state.setAngularVelocityZ(rate);
            };
            ((ShamSwerve) simCtx.robot().getDriveTrain())
                    .getGyro().setYawVeloConsumer(simapplier);
        }

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
                rollSignal, pitchSignal,
                rollVeloSignal, pitchVeloSignal);

        super.pitchRads = Units.degreesToRadians(pitchSignal.getValueAsDouble());
        super.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValueAsDouble());
        super.rollRads = Units.degreesToRadians(rollSignal.getValueAsDouble());
        super.rollVelRadsPerSec = Units.degreesToRadians(rollVeloSignal.getValueAsDouble());
        super.yawRads = odoThread.getGyroYaw();
        super.yawVelRadsPerSec = odoThread.getGyroYawRate();
    }
}
