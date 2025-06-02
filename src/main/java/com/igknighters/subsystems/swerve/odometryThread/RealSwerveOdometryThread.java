package igknighters.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import igknighters.constants.ConstValues;
import igknighters.subsystems.swerve.SwerveConstants.ModuleConstants.kWheel;
import igknighters.util.plumbing.Channel.Sender;
import java.util.concurrent.atomic.AtomicLong;

public class RealSwerveOdometryThread extends SwerveOdometryThread {
  private final Thread thread;
  private final BaseStatusSignal[] signals = new BaseStatusSignal[(MODULE_COUNT * 4) + 4];

  protected final MedianFilter peakRemover = new MedianFilter(3);
  protected final LinearFilter lowPass = LinearFilter.movingAverage(50);

  /** An array that holds [module1Pos, module1Velo, module2Pos, ...] */
  protected final AtomicLong[] moduleStates = new AtomicLong[MODULE_COUNT * 2];

  protected final AtomicLong[] gyroStates = new AtomicLong[2];

  protected boolean enableLatencyCompensation = false;

  private double getAtomicDouble(AtomicLong[] array, int index) {
    return Double.longBitsToDouble(array[index].get());
  }

  public RealSwerveOdometryThread(int hz, Sender<SwerveDriveSample> swerveDataSender) {
    super(hz, swerveDataSender);
    this.thread = new Thread(this::run, "OdometryThread");
    for (int i = 0; i < MODULE_COUNT * 2; i++) {
      moduleStates[i] = new AtomicLong();
    }
    gyroStates[0] = new AtomicLong();
    gyroStates[1] = new AtomicLong();
  }

  private double latencyCompensatedValue(BaseStatusSignal signal, BaseStatusSignal signalSlope) {
    if (!enableLatencyCompensation) {
      return signal.getValueAsDouble();
    }
    final double maxLatencySeconds = ConstValues.PERIODIC_TIME * 5;
    final double nonCompensatedSignal = signal.getValueAsDouble();
    final double changeInSignal = signalSlope.getValueAsDouble();
    double latency = signal.getTimestamp().getLatency();
    if (maxLatencySeconds > 0.0 && latency > maxLatencySeconds) {
      latency = maxLatencySeconds;
    }
    return nonCompensatedSignal + (changeInSignal * latency);
  }

  public void addModuleStatusSignals(
      int moduleId,
      BaseStatusSignal drivePosition,
      BaseStatusSignal driveVelocity,
      BaseStatusSignal anglePosition,
      BaseStatusSignal angleVelocity) {
    BaseStatusSignal.setUpdateFrequencyForAll(
        hz, drivePosition, driveVelocity, anglePosition, angleVelocity);
    int offset = 4 * moduleId;
    signals[offset + 0] = drivePosition;
    signals[offset + 1] = driveVelocity;
    signals[offset + 2] = anglePosition;
    signals[offset + 3] = angleVelocity;
  }

  public void addGyroStatusSignals(
      BaseStatusSignal yaw,
      BaseStatusSignal yawRate,
      BaseStatusSignal xAccel,
      BaseStatusSignal yAccel) {
    BaseStatusSignal.setUpdateFrequencyForAll(hz, yaw, yawRate, xAccel, yAccel);
    signals[MODULE_COUNT * 4] = yaw;
    signals[(MODULE_COUNT * 4) + 1] = yawRate;
    signals[(MODULE_COUNT * 4) + 2] = xAccel;
    signals[(MODULE_COUNT * 4) + 3] = yAccel;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      int offset = 4 * i;
      positions[i] =
          new SwerveModulePosition(
              latencyCompensatedValue(signals[offset + 0], signals[offset + 1])
                  * kWheel.CIRCUMFERENCE,
              Rotation2d.fromRotations(
                  latencyCompensatedValue(signals[offset + 2], signals[offset + 3])));
    }
    return positions;
  }

  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(
        latencyCompensatedValue(signals[MODULE_COUNT * 4], signals[(MODULE_COUNT * 4) + 1]));
  }

  private double getDataLatency() {
    double totalLatency = 0.0;
    for (var signal : signals) {
      totalLatency += signal.getTimestamp().getLatency();
    }
    return totalLatency / (double) signals.length;
  }

  private double getAcceleration() {
    return Math.hypot(
        signals[(MODULE_COUNT * 4) + 2].getValueAsDouble(),
        signals[(MODULE_COUNT * 4) + 3].getValueAsDouble());
  }

  private void run() {
    isRunning.set(true);
    try {
      Threads.setCurrentThreadPriority(true, 1);

      while (this.isRunning.get()) {
        long startTime = RobotController.getFPGATime();
        BaseStatusSignal.waitForAll(2.0 / hz, signals);
        long elapsedTime = RobotController.getFPGATime() - startTime;

        updateTimeMicros.set((long) lowPass.calculate(peakRemover.calculate(elapsedTime)));

        if (DriverStation.isTestEnabled()) {
          continue;
        }

        for (int i = 0; i < MODULE_COUNT; i++) {
          int positionOffset = 4 * i;
          int veloOffset = positionOffset + 1;

          moduleStates[i * 2].set(
              Double.doubleToLongBits(signals[positionOffset].getValueAsDouble()));
          moduleStates[(i * 2) + 1].set(
              Double.doubleToLongBits(signals[veloOffset].getValueAsDouble()));
        }

        gyroStates[0].set(
            Double.doubleToLongBits(
                Units.degreesToRadians(signals[signals.length - 4].getValueAsDouble())));
        gyroStates[1].set(
            Double.doubleToLongBits(
                Units.degreesToRadians(signals[signals.length - 3].getValueAsDouble())));

        swerveDataSender.send(
            new SwerveDriveSample(
                getModulePositions(),
                getGyroRotation(),
                getAcceleration(),
                Timer.getFPGATimestamp() - log("latency", getDataLatency())));
      }
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), e.getStackTrace());
    } finally {
      isRunning.set(false);
    }
  }

  @Override
  public void start() {
    thread.start();
  }

  public double getModulePosition(int moduleId) {
    return getAtomicDouble(moduleStates, moduleId * 2);
  }

  public double getModuleVelocity(int moduleId) {
    return getAtomicDouble(moduleStates, (moduleId * 2) + 1);
  }

  public double getGyroYaw() {
    return getAtomicDouble(gyroStates, 0);
  }

  public double getGyroYawRate() {
    return getAtomicDouble(gyroStates, 1);
  }
}
