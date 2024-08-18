package com.igknighters.subsystems.swerve.odometryThread;

import java.util.concurrent.atomic.AtomicLong;
import java.util.function.DoubleFunction;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;

public class RealSwerveOdometryThread extends SwerveOdometryThread {
    private final Thread thread;
    private final BaseStatusSignal[] signals = new BaseStatusSignal[(MODULE_COUNT * 4) + 2];
    private final DoubleFunction<Double> driveRotsToMeters;

    protected final MedianFilter peakRemover = new MedianFilter(3);
    protected final LinearFilter lowPass = LinearFilter.movingAverage(50);

    /** An array that holds [module1Pos, module1Velo, module2Pos, ...] */
    protected final AtomicLong[] moduleStates = new AtomicLong[MODULE_COUNT * 2];

    private double getAtomicDouble(AtomicLong[] array, int index) {
        return Double.longBitsToDouble(array[index].get());
    }

    public RealSwerveOdometryThread(int hz, DoubleFunction<Double> driveRotsToMeters) {
        super(hz);
        this.thread = new Thread(this::run, "OdometryThread");
        this.driveRotsToMeters = driveRotsToMeters;
    }

    public void addModuleStatusSignals(
        int moduleId,
        StatusSignal<Double> drivePosition,
        StatusSignal<Double> driveVelocity,
        StatusSignal<Double> anglePosition,
        StatusSignal<Double> angleVelocity
    ) {
        BaseStatusSignal.setUpdateFrequencyForAll(
            hz,
            drivePosition,
            driveVelocity,
            anglePosition,
            angleVelocity
        );
        int offset = 4 * moduleId;
        signals[offset + 0] = drivePosition;
        signals[offset + 1] = driveVelocity;
        signals[offset + 2] = anglePosition;
        signals[offset + 3] = angleVelocity;
    }

    public void addGyroStatusSignals(
        StatusSignal<Double> yaw,
        StatusSignal<Double> yawRate
    ) {
        BaseStatusSignal.setUpdateFrequencyForAll(
            hz,
            yaw,
            yawRate
        );
        signals[signals.length - 2] = yaw;
        signals[signals.length - 1] = yawRate;
    }

    @SuppressWarnings("unchecked")
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT; i++) {
            int offset = 4 * i;
            positions[i] = new SwerveModulePosition(
                driveRotsToMeters.apply(
                    BaseStatusSignal.getLatencyCompensatedValue(
                        (StatusSignal<Double>) signals[offset + 0],
                        (StatusSignal<Double>) signals[offset + 1]
                    )
                ),
                Rotation2d.fromRotations(
                    BaseStatusSignal.getLatencyCompensatedValue(
                        (StatusSignal<Double>) signals[offset + 2],
                        (StatusSignal<Double>) signals[offset + 3]
                    )
                )
            );
        }
        return positions;
    }

    @SuppressWarnings("unchecked")
    private Rotation2d getGyroRotation() {
        return Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValue(
                (StatusSignal<Double>) signals[signals.length - 2],
                (StatusSignal<Double>) signals[signals.length - 1]
            )
        );
    }

    private void run() {
        Threads.setCurrentThreadPriority(true, 1);

        while (this.isRunning.get()) {
            long startTime = RobotController.getFPGATime();
            BaseStatusSignal.waitForAll(2.0 / hz, signals);
            long elapsedTime = RobotController.getFPGATime() - startTime;

            updateTimeMicros.set(
                (long) lowPass.calculate(
                    peakRemover.calculate(
                        elapsedTime
                    )
                )
            );

            for (int i = 0; i < MODULE_COUNT; i++) {
                int positionOffset = 4 * i;
                int veloOffset = positionOffset + 1;

                moduleStates[i * 2].set(Double.doubleToLongBits(signals[positionOffset].getValueAsDouble()));
                moduleStates[(i * 2) + 1].set(Double.doubleToLongBits(signals[veloOffset].getValueAsDouble()));
            }

            swerveDataSender.send(
                new SwerveDriveSample(
                    new SwerveDriveWheelPositions(getModulePositions()),
                    getGyroRotation(),
                    MathSharedStore.getTimestamp()
                )
            );
        }
    }

    @Override
    public void start() {
        isRunning.set(true);
        thread.start();
    }

    public double getModulePosition(int moduleId) {
        return getAtomicDouble(moduleStates, moduleId * 2);
    }

    public double getModuleVelocity(int moduleId) {
        return getAtomicDouble(moduleStates, (moduleId * 2) + 1);
    }
}
