package igknighters.subsystems.swerve.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kRotationController;
import igknighters.subsystems.swerve.Swerve;
import monologue.ProceduralStructGenerator;

public class RotationalController {
  private final Swerve swerve;

  private double positionError = 0, prevError = 0, velocityError = 0;

  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              kSwerve.MAX_ANGULAR_VELOCITY * kRotationController.CONSTRAINT_SCALAR,
              kSwerve.MAX_ANGULAR_ACCELERATION * kRotationController.CONSTRAINT_SCALAR));
  private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  public RotationalController(Swerve swerve) {
    this.swerve = swerve;
  }

  // OBJ_COUNT: 4
  public double calculate(double target, double deadband) {
    double measurement = MathUtil.angleModulus(swerve.getYawRads());

    if (Math.abs(measurement - target) < deadband) {
      var output =
          new RotationalControllerOutput(
              target, measurement, true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      swerve.log("RotationalControllerOutput", output);
      return 0.0;
    }

    // set fields individually to prevent new object
    goalState.position = target;
    goalState.velocity = 0.0;

    double goalMinDistance = MathUtil.angleModulus(goalState.position - measurement);
    double setpointMinDistance = MathUtil.angleModulus(setpointState.position - measurement);

    goalState.position = goalMinDistance + measurement;
    setpointState.position = setpointMinDistance + measurement;

    setpointState = profile.calculate(ConstValues.PERIODIC_TIME, setpointState, goalState);

    prevError = positionError;

    positionError = MathUtil.angleModulus(setpointState.position - measurement);

    velocityError = (positionError - prevError) / ConstValues.PERIODIC_TIME;

    double rotVelo =
        (kRotationController.kP * positionError) + (kRotationController.kD * velocityError);

    var output =
        new RotationalControllerOutput(
            target,
            measurement,
            false,
            goalMinDistance,
            setpointMinDistance,
            setpointState.velocity,
            setpointState.position,
            positionError,
            velocityError,
            rotVelo);

    swerve.log("RotationalControllerOutput", output);

    return rotVelo;
  }

  public void reset() {
    positionError = 0;
    prevError = 0;
    velocityError = 0;
    setpointState =
        new TrapezoidProfile.State(swerve.getYawRads(), swerve.getRobotSpeeds().omega());
  }

  public static record RotationalControllerOutput(
      double target,
      double measurement,
      boolean deadband,
      double goalMinDistance,
      double setpointMinDistance,
      double setpointStateVelocity,
      double setpointStatePosition,
      double positionError,
      double velocityError,
      double rotationalVelocity)
      implements StructSerializable {
    public static final Struct<RotationalControllerOutput> struct =
        ProceduralStructGenerator.genRecord(RotationalControllerOutput.class);
  }
}
