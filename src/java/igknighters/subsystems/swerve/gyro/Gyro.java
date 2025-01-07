package igknighters.subsystems.swerve.gyro;

import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class Gyro extends Component {
  @Log protected double pitchRads = 0.0;
  @Log protected double rollRads = 0.0;
  @Log protected double yawRads = 0.0;
  @Log protected double pitchVelRadsPerSec = 0.0;
  @Log protected double rollVelRadsPerSec = 0.0;
  @Log protected double yawVelRadsPerSec = 0.0;
  @Log protected double accelX = 0.0;
  @Log protected double accelY = 0.0;

  public double getPitchRads() {
    return pitchRads;
  }

  public double getRollRads() {
    return rollRads;
  }

  public double getYawRads() {
    return yawRads;
  }

  public abstract void setYawRads(double yawRads);
}
