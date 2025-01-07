package sham.configs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.ProceduralStructGenerator;

public class ShamGyroConfig implements StructSerializable {
  public double averageDriftingIn30SecsMotionlessDeg;
  public double velocityMeasurementStandardDeviationPercent;

  public ShamGyroConfig(
      double averageDriftingIn30SecsMotionlessDeg,
      double velocityMeasurementStandardDeviationPercent) {
    this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
    this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
  }

  public ShamGyroConfig() {}

  public ShamGyroConfig withAverageDriftingIn30SecsMotionlessDeg(
      double averageDriftingIn30SecsMotionlessDeg) {
    this.averageDriftingIn30SecsMotionlessDeg = averageDriftingIn30SecsMotionlessDeg;
    return this;
  }

  public ShamGyroConfig withVelocityMeasurementStandardDeviationPercent(
      double velocityMeasurementStandardDeviationPercent) {
    this.velocityMeasurementStandardDeviationPercent = velocityMeasurementStandardDeviationPercent;
    return this;
  }

  /**
   *
   *
   * <h2>Creates the Simulation for a <a href="https://store.ctr-electronics.com/pigeon-2/">CTRE
   * Pigeon 2 IMU</a>. </h2>
   *
   * @return a gyro simulation factory configured for the Pigeon 2 IMU
   */
  public static ShamGyroConfig ofPigeon2() {
    /*
     * user manual of pigeon 2:
     * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
     */
    return new ShamGyroConfig(0.5, 0.02);
  }

  /**
   *
   *
   * <h2>Creates the Simulation for a <a href="https://pdocs.kauailabs.com/navx-mxp/">navX2-MXP
   * IMU</a>.</h2>
   *
   * @return a gyro simulation factory configured for the navX2-MXP IMU
   */
  public static ShamGyroConfig ofNavX2() {
    return new ShamGyroConfig(2, 0.04);
  }

  public static final Struct<ShamGyroConfig> struct =
      ProceduralStructGenerator.genObjectNoUnpack(ShamGyroConfig.class);
}
