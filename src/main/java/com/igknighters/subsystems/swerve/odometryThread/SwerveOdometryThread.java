package com.igknighters.subsystems.swerve.odometryThread;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import com.igknighters.util.plumbing.Channel.Sender;

import monologue.Logged;
import monologue.Annotations.Log;

public abstract class SwerveOdometryThread implements Logged {
    protected final static int MODULE_COUNT = 4;

    protected final int hz;

    protected final AtomicBoolean isRunning = new AtomicBoolean(false);
    protected final AtomicLong updateTimeMicros = new AtomicLong();

    protected final Sender<SwerveDriveSample> swerveDataSender;

    protected SwerveOdometryThread(int hz, Sender<SwerveDriveSample> swerveDataSender) {
        this.hz = hz;
        this.swerveDataSender = swerveDataSender;
    }

    @Log
    private double updateTimeMili() {
        return updateTimeMicros.get() / 1_000.0;
    }

    @Log
    private boolean isRunning() {
        return isRunning.get();
    }

    public abstract void start();
}
