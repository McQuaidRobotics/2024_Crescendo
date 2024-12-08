package sham;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Force;
import monologue.Monologue;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.dyn4j.geometry.Vector2;
import sham.ShamArena.ShamEnvTiming;
import sham.ShamMechanism.MechanismDynamics;
import sham.ShamMechanism.MechanismInputs;
import sham.ShamMechanism.MechanismOutputs;
import sham.configs.ShamSwerveConfig;
import sham.configs.ShamSwerveModuleConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.MeasureMath;

public class ShamDriveTrainSwerveModule {
    private final ShamMechanism driveMotor;
    private final ShamMechanism steerMotor;
    public final double wheelsCoefficientOfFriction,
            wheelRadiusMeters;

    private final ShamEnvTiming timing;

    private final int moduleId;

    /**
     *
     *
     * <h2>Constructs a Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link ShamArena#overrideSimulationTimings(double, int)} to use custom timings, you must
     * call the method before constructing any swerve module simulations using this constructor.
     *
     * @param driveMotor the model of the driving motor
     * @param steerMotor the model of the steering motor
     * @param driveCurrentLimit the current limit for the driving motor, in amperes
     * @param driveGearRatio the gear ratio for the driving motor, >1 is reduction
     * @param steerGearRatio the gear ratio for the steering motor, >1 is reduction
     * @param driveFrictionVoltage the measured minimum amount of voltage that can turn the driving rotter, in volts
     * @param steerFrictionVoltage the measured minimum amount of voltage that can turn the steering rotter, in volts
     * @param tireCoefficientOfFriction the <a
     *     href='https://simple.wikipedia.org/wiki/Coefficient_of_friction#:~:text=A%20coefficient%20of%20friction%20is%20a%20value%20that%20shows%20the'>coefficient
     *     of friction</a> of the tires, normally around 1.5
     * @param wheelsRadiusMeters the radius of the wheels, in meters. Calculate it using
     *     {@link Units#inchesToMeters(double)}.
     * @param steerRotationalInertia the rotational inertia of the steering mechanism
     */
    ShamDriveTrainSwerveModule(ShamRobot robot, ShamSwerveConfig config, int moduleId, ShamMotorController driveController, ShamMotorController steerController) {
        final ShamSwerveModuleConfig moduleConfig = config.swerveModuleConfig;
        timing = robot.timing();
        driveMotor = new ShamMechanism(
            "SwerveModuleDriveMotor" + moduleId,
            moduleConfig.driveConfig.motor,
            driveController,
            moduleConfig.driveConfig.rotorInertia,
            moduleConfig.driveConfig.gearRatio,
            moduleConfig.driveConfig.friction,
            MechanismDynamics.zero(),
            moduleConfig.driveConfig.limits,
            moduleConfig.driveConfig.noise,
            timing
        );
        steerMotor = new ShamMechanism(
            "SwerveModuleSteerMotor" + moduleId,
            moduleConfig.steerConfig.motor,
            steerController,
            moduleConfig.steerConfig.rotorInertia,
            moduleConfig.steerConfig.gearRatio,
            moduleConfig.steerConfig.friction,
            MechanismDynamics.zero(),
            moduleConfig.steerConfig.limits,
            moduleConfig.steerConfig.noise,
            timing
        );
        robot.addMechanism(driveMotor);
        robot.addMechanism(steerMotor);
        wheelsCoefficientOfFriction = config.swerveModuleConfig.tireCoefficientOfFriction;
        wheelRadiusMeters = config.swerveModuleConfig.wheelsRadiusMeters;
        this.moduleId = moduleId;

        RuntimeLog.debug("Created a swerve module simulation");
    }

    public record ModuleMotorPair<T>(T drive, T steer) {
    }

    public ModuleMotorPair<MechanismInputs> inputs() {
        return new ModuleMotorPair<>(driveMotor.inputs(), steerMotor.inputs());
    }

    public ModuleMotorPair<MechanismOutputs> outputs() {
        return new ModuleMotorPair<>(driveMotor.outputs(), steerMotor.outputs());
    }

    public SwerveModuleState state() {
        return new SwerveModuleState(
            driveMotor.outputs().velocity().in(RadiansPerSecond) * wheelRadiusMeters,
            new Rotation2d(steerMotor.outputs().position())
        );
    }

    protected Vector2 moduleForce(final double gravityForceOnModuleNewtons) {
        final Force driveMotorAppliedForce = driveMotor.inputs().torque().divide(Meters.of(wheelRadiusMeters));
        final Force gripForce = Newtons.of(gravityForceOnModuleNewtons * wheelsCoefficientOfFriction);
        final Angle steerMotorAngle = steerMotor.outputs().position();

        final boolean isSkidding = MeasureMath.abs(driveMotorAppliedForce).gt(gripForce);
        final Force propellingForce;
        if (isSkidding) {
            propellingForce = gripForce.times(MeasureMath.signum(driveMotorAppliedForce));
        } else {
            propellingForce = driveMotorAppliedForce;
        }

        Monologue.log(moduleId+"ModuleForce", propellingForce.baseUnitMagnitude());

        return Vector2.create(
            propellingForce.in(Newtons),
            steerMotorAngle.in(Radians)
        );
    }
}
