package igknighters.subsystems.swerve.odometryThread;

import igknighters.util.plumbing.Channel.Sender;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import monologue.Annotations.Log;
import monologue.Logged;

public abstract class SwerveOdometryThread implements Logged {
  protected static final int MODULE_COUNT = 4;

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
