package com.igknighters.subsystems.swerve.odometryThread;

import java.util.function.Supplier;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;

public class SimSwerveOdometryThread extends SwerveOdometryThread{
    private final Notifier notifier;

    @SuppressWarnings("unchecked")
    private final Supplier<SwerveModulePosition>[] positiionSuppliers = new Supplier[MODULE_COUNT];
    private Supplier<Rotation2d> rotationSupplier = Rotation2d::new;

    public SimSwerveOdometryThread(int hz) {
        super(hz);
        notifier = new Notifier(this::run);
    }

    public void addModulePositionSupplier(int moduleId, Supplier<SwerveModulePosition> sup) {
        positiionSuppliers[moduleId] = sup;
    }

    public void addRotationSupplier(Supplier<Rotation2d> sup) {
        rotationSupplier = sup;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT; i++) {
            positions[i] = positiionSuppliers[i].get();
        }
        return positions;
    }

    private void run() {
        swerveDataSender.send(
            new SwerveDriveSample(
                new SwerveDriveWheelPositions(getModulePositions()),
                rotationSupplier.get(),
                MathSharedStore.getTimestamp()
            )
        );
    }

    @Override
    public void start() {
        notifier.startPeriodic(1.0 / ((double) hz));
    }
}
