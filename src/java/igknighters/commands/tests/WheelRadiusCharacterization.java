package igknighters.commands.tests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import wpilibExt.Speeds;

public class WheelRadiusCharacterization extends Command {
  private static final TunableDouble characterizationSpeed =
      TunableValues.getDouble("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
  private static final double driveRadius = kSwerve.DRIVEBASE_RADIUS;

  private final DoubleSupplier gyroYawRadsSupplier;
  private final ChassisSpeeds outputSpeed = new ChassisSpeeds();

  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    Direction(int value) {
      this.value = value;
    }
  }

  private final Swerve swerve;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  private double[] wheelPositions() {
    return Arrays.stream(swerve.getModulePositions())
        .mapToDouble(s -> s.distanceMeters / kSwerve.WHEEL_CIRCUMFERENCE)
        .toArray();
  }

  public WheelRadiusCharacterization(Swerve swerve, Direction omegaDirection) {
    this.swerve = swerve;
    this.omegaDirection = omegaDirection;
    this.gyroYawRadsSupplier = swerve::getYawRads;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = wheelPositions();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    outputSpeed.omegaRadiansPerSecond =
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.value());
    swerve.drive(Speeds.fromRobotRelative(outputSpeed));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = wheelPositions();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;

    swerve.log("RadiusCharacterization/DrivePosition", averageWheelPosition);
    swerve.log("RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    swerve.log(
        "RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
