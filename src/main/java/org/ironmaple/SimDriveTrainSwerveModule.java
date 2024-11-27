package org.ironmaple;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Force;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.SimArena.SimulationTiming;
import org.ironmaple.SimMechanism.MechanismDynamics;
import org.ironmaple.SimMechanism.MechanismInputs;
import org.ironmaple.SimMechanism.MechanismOutputs;
import org.ironmaple.configs.SwerveConfig;
import org.ironmaple.configs.SwerveModuleConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.MeasureMath;

public class SimDriveTrainSwerveModule {
    private final SimMechanism driveMotor;
    private final SimMechanism steerMotor;
    public final double wheelsCoefficientOfFriction,
            wheelRadiusMeters;

    private final SimulationTiming timing;

    /**
     *
     *
     * <h2>Constructs a Swerve Module Simulation.</h2>
     *
     * <p>If you are using {@link SimArena#overrideSimulationTimings(double, int)} to use custom timings, you must
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
    SimDriveTrainSwerveModule(SimRobot robot, SwerveConfig config, int moduleId, SimMotorController driveController, SimMotorController steerController) {
        final SwerveModuleConfig moduleConfig = config.swerveModuleConfig;
        timing = robot.timing();
        driveMotor = new SimMechanism(
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
        steerMotor = new SimMechanism(
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
            new Rotation2d(steerMotor.outputs().angle())
        );
    }

    Vector2 updateSimulationSubTickGetModuleForce(double gravityForceOnModuleNewtons) {
        Force driveMotorAppliedForce = driveMotor.inputs().torque().divide(Meters.of(wheelRadiusMeters));
        Force gripForce = Newtons.of(gravityForceOnModuleNewtons * wheelsCoefficientOfFriction);
        Angle steerMotorAppliedAngle = steerMotor.outputs().angle();

        boolean isSkidding = MeasureMath.abs(driveMotorAppliedForce).gt(gripForce);
        Force propellingForce;
        if (isSkidding) {
            propellingForce = gripForce.times(MeasureMath.signum(driveMotorAppliedForce));
        } else {
            propellingForce = driveMotorAppliedForce;
        }

        return Vector2.create(propellingForce.in(Newtons), steerMotorAppliedAngle.in(Radians));
    }
}
