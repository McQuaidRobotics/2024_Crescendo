package com.igknighters.subsystems.swerve;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.kRotationController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RotationalController {
    private final Swerve swerve;

    private double positionError = 0, prevError = 0, velocityError = 0;

    private final TrapezoidProfile profile = new TrapezoidProfile(
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

        return (kRotationController.kP * positionError)
                + (kRotationController.kD * velocityError);
    }

    public void reset() {
        positionError = 0;
        prevError = 0;
        velocityError = 0;
        setpointState = new TrapezoidProfile.State(swerve.getYawRads(), swerve.getChassisSpeed().omegaRadiansPerSecond);
    }
}
