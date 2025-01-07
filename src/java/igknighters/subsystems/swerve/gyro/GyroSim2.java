package igknighters.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import sham.ShamGyro;

public class GyroSim2 extends Gyro {

  public GyroSim2(ShamGyro simGyro, SimSwerveOdometryThread odoThread) {
    simGyro.setUpdateConsumer(
        (yawPair, accelVector) -> {
          super.yawRads = yawPair.getFirst().in(Radians);
          super.yawVelRadsPerSec = yawPair.getSecond().in(RadiansPerSecond);
          super.accelX = accelVector.x().in(MetersPerSecondPerSecond);
          super.accelY = accelVector.y().in(MetersPerSecondPerSecond);
        });

    odoThread.addRotationSupplier(() -> Rotation2d.fromRadians(this.getYawRads()));
    odoThread.addAccelerationSupplier(() -> new double[] {this.accelX, this.accelY});

    BootupLogger.bootupLog("    Gyro initialized (sim)");
  }

  @Override
  public void setYawRads(double yawRads) {
    super.yawRads = yawRads;
  }
}
