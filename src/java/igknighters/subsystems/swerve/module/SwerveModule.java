package igknighters.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.RobotConfig;
import igknighters.subsystems.Component;
import monologue.Annotations.Log;

public abstract class SwerveModule extends Component {
  public static final class AdvancedSwerveModuleState extends SwerveModuleState {
    public double steerVelocityFF;
    public double driveAccelerationFF;

    public AdvancedSwerveModuleState(
        double speedMetersPerSecond,
        Rotation2d angle,
        double steerVelocityFF,
        double driveAccelerationFF) {
      super(speedMetersPerSecond, angle);
      this.steerVelocityFF = steerVelocityFF;
      this.driveAccelerationFF = driveAccelerationFF;
    }

    // todo: implement custom struct
    public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

    public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
      return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0, 0.0);
    }
  }

  @Log public double driveVeloMPS = 0.0;
  @Log public double targetDriveVeloMPS = 0.0;
  @Log public double drivePositionMeters = 0.0;
  @Log public double driveVolts = 0.0;
  @Log public double driveAmps = 0.0;
  @Log public double steerVeloRadPS = 0.0;
  @Log public double steerAbsoluteRads = 0.0;
  @Log public double targetSteerAbsoluteRads = 0.0;
  @Log public double steerVolts = 0.0;
  @Log public double steerAmps = 0.0;

  public final String name;

  protected SwerveModule(String name) {
    this.name = name;
  }

  /**
   * @param desiredState The state that the module should assume, angle and velocity.
   * @param isOpenLoop Whether the module speed assumed should be reached via open or closed loop
   *     control.
   */
  public abstract void setDesiredState(AdvancedSwerveModuleState desiredState);

  /**
   * @return The velocity/angle of the module.
   */
  public abstract SwerveModuleState getCurrentState();

  public abstract SwerveModulePosition getCurrentPosition();

  public abstract int getModuleId();

  public abstract void setVoltageOut(double volts, Rotation2d angle);

  protected double getOffset(int moduleId) {
    double[] offsetStore =
        switch (RobotConfig.getRobotID()) {
          case CRASH -> kSwerve.CRASH_ROTATION_OFFSETS;
          default -> kSwerve.BURN_ROTATION_OFFSETS;
        };

    return offsetStore[moduleId];
  }
}
