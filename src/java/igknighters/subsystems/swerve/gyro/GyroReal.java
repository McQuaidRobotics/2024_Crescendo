package igknighters.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.HardwareIndex.SwerveHW;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.util.can.CANRetrier;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.BootupLogger;
import igknighters.util.logging.FaultManager;
import monologue.Annotations.IgnoreLogged;

public class GyroReal extends Gyro {

  private final Pigeon2 gyro;
  private final BaseStatusSignal rollSignal, pitchSignal;
  private final BaseStatusSignal rollVeloSignal, pitchVeloSignal;

  @IgnoreLogged private final RealSwerveOdometryThread odoThread;

  public GyroReal(RealSwerveOdometryThread odoThread) {
    this.odoThread = odoThread;

    gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
    CANRetrier.retryStatusCode(() -> gyro.getConfigurator().apply(new Pigeon2Configuration()), 5);

    rollSignal = gyro.getRoll();
    pitchSignal = gyro.getPitch();

    pitchVeloSignal = gyro.getAngularVelocityXWorld();
    rollVeloSignal = gyro.getAngularVelocityYWorld();

    CANSignalManager.registerSignals(
        kSwerve.CANBUS, rollSignal, pitchSignal, rollVeloSignal, pitchVeloSignal);

    odoThread.addGyroStatusSignals(
        gyro.getYaw(),
        gyro.getAngularVelocityZWorld(),
        gyro.getAccelerationX(),
        gyro.getAccelerationY());

    gyro.optimizeBusUtilization(0.0, 1.0);

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
        SwerveHW.Pigeon2, rollSignal, pitchSignal, rollVeloSignal, pitchVeloSignal);

    super.pitchRads = Units.degreesToRadians(pitchSignal.getValueAsDouble());
    super.pitchVelRadsPerSec = Units.degreesToRadians(pitchVeloSignal.getValueAsDouble());
    super.rollRads = Units.degreesToRadians(rollSignal.getValueAsDouble());
    super.rollVelRadsPerSec = Units.degreesToRadians(rollVeloSignal.getValueAsDouble());
    super.yawRads = odoThread.getGyroYaw();
    super.yawVelRadsPerSec = odoThread.getGyroYawRate();
  }
}
