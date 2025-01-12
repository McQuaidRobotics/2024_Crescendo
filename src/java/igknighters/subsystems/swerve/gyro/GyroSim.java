package igknighters.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.constants.ConstValues;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import java.util.function.Supplier;
import wpilibExt.Speeds.RobotSpeeds;

public class GyroSim extends Gyro {

  private final Supplier<RobotSpeeds> chassisSpeedSupplier;

  public GyroSim(Supplier<RobotSpeeds> chassisSpeedSupplier, SimSwerveOdometryThread odoThread) {
    this.chassisSpeedSupplier = chassisSpeedSupplier;

    odoThread.addRotationSupplier(() -> Rotation2d.fromRadians(this.getYawRads()));

    BootupLogger.bootupLog("    Gyro initialized (sim)");
  }

  @Override
  public double getPitchRads() {
    return 0.0;
  }

  @Override
  public double getRollRads() {
    return 0.0;
  }

  @Override
  public double getYawRads() {
    return super.yawRads;
  }

  @Override
  public void setYawRads(double yawRads) {
    super.yawRads = yawRads;
  }

  @Override
  public void periodic() {
    var oldYaw = super.yawRads;
    super.yawRads += chassisSpeedSupplier.get().omega() * ConstValues.PERIODIC_TIME;
    super.yawVelRadsPerSec = (super.yawRads - oldYaw) / ConstValues.PERIODIC_TIME;
  }
}
