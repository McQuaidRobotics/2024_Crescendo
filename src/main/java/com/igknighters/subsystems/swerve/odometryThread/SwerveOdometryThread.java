package com.igknighters.subsystems.swerve.odometryThread;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.util.plumbing.Channels.Sender;

import monologue.Logged;
import monologue.Annotations.Log;

public abstract class SwerveOdometryThread implements Logged {
    protected final static int MODULE_COUNT = 4;

    protected final int hz;

    protected final AtomicBoolean isRunning = new AtomicBoolean(false);
    protected final AtomicLong updateTimeMicros = new AtomicLong();

    protected final Sender<SwerveDriveSample> swerveDataSender = Sender.broadcast(kChannels.SWERVE_ODO_SAMPLES, SwerveDriveSample.class);

    protected SwerveOdometryThread(int hz) {
        this.hz = hz;
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
