package sham;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static sham.utils.mathutils.MeasureMath.div;

import java.util.Random;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import sham.ShamArena.ShamEnvTiming;
import sham.ShamMotorController.ControllerOutput;
import sham.utils.DCMotor2;
import sham.utils.GearRatio;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.MeasureMath;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import monologue.ProceduralStructGenerator;

public class ShamMechanism {
    private static final double kMotorEfficiency = 0.85;

    private static final Random RAND = new Random();

    private static final AngleUnit Rad = Radians;
    private static final AngularVelocityUnit RadPS = RadiansPerSecond;
    private static final AngularAccelerationUnit RadPS2 = RadiansPerSecondPerSecond;

    /**
     * An interface to define systemic impacts on a mechanism.
     * 
     * <p>This could be the force of gravity, a spring, inertial load due to chassis
     * acceleration, etc.
     * 
     * <p>It is sound behavior for this to capture outputs of other mechanisms but
     * be aware the order of calculations between mechanisms is not guaranteed.
     * For example, if an inverse pendulum mechanism could have extra load depending
     * on the drive mechanism's acceleration, the drive mechanism should be calculated
     * first in the same time slot but this can not be promised.
     */
    public interface MechanismDynamics {
        default Torque environment(MechanismState state) {
            return NewtonMeters.zero();
        }

        default MomentOfInertia extraInertia() {
            return KilogramSquareMeters.zero();
        }

        static MechanismDynamics of(Torque environment) {
            return new MechanismDynamics() {
                @Override
                public Torque environment(MechanismState state) {
                    return environment;
                }

                @Override
                public MomentOfInertia extraInertia() {
                    return KilogramSquareMeters.zero();
                }
            };
        }

        static MechanismDynamics zero() {
            return MechanismDynamics.of(NewtonMeters.zero());
        }
    }

    /**
     * Defines the friction of a mechanism.
     * The only way to obtain this value is to empirically measure it.
     * This record allows defining separate static and kinetic friction values
     * but it is perfectly valid to use the same value for both.
     * 
     * <p> Creating a friction using {@code kS} is a valid approach and can
     * be done like so:
     * <pre><code>
     * //could be any motor
     *DCMotor motor = DCMotor.getFalcon500(1);
     *Friction.of(motor, Volts.of(kMechanism.kS));
     * </code></pre>
     */
    public record Friction(Torque staticFriction, Torque kineticFriction) implements StructSerializable {
        public static Friction of(Torque staticFriction, Torque kineticFriction) {
            return new Friction(staticFriction, kineticFriction);
        }

        /**
         * Constructs a {@link Friction} object with the same value for both static and kinetic friction.
         *
         * @param universalFriction the value to use for both static and kinetic friction
         * @return a {@link Friction} object with the same value for both static and kinetic friction
         */
        public static Friction of(Torque universalFriction) {
            return new Friction(universalFriction, universalFriction);
        }

        /**
         * Constructs a {@link Friction} object using a {@link DCMotor} object and voltage friction constants.
         * 
         * @param motor the motor to use for the friction calculation
         * @param staticVoltage the voltage to use for the static friction calculation
         * @param kineticVoltage the voltage to use for the kinetic friction calculation
         * @return a {@link Friction} object using the given motor and voltage constants
         */
        public static Friction of(DCMotor motor, Voltage staticVoltage, Voltage kineticVoltage) {
            Torque staticTorque = NewtonMeters.of(
                motor.getTorque(motor.getCurrent(0.0, staticVoltage.in(Volts))));
            Torque kineticTorque = NewtonMeters.of(
                motor.getTorque(motor.getCurrent(0.0, kineticVoltage.in(Volts))));
            return new Friction(staticTorque, kineticTorque);
        }

        public static Friction of(DCMotor motor, Voltage universalVoltage) {
            return of(motor, universalVoltage, universalVoltage);
        }

        public static Friction zero() {
            return new Friction(NewtonMeters.zero(), NewtonMeters.zero());
        }

        public static final Struct<Friction> struct = ProceduralStructGenerator.genRecord(Friction.class);
    }

    /**
     * Defines the mechanical positional limits of the mechanism.
     * 
     * @see #unbounded() HardLimits.unbounded() for a mechanism with no limits
     */
    public record HardLimits(Angle minAngle, Angle maxAngle) implements StructSerializable {
        public static HardLimits of(Angle minAngle, Angle maxAngle) {
            return new HardLimits(minAngle, maxAngle);
        }

        /**
         * Constructs a {@link HardLimits} object with no limits.
         * 
         * @return a {@link HardLimits} object with no limits
         */
        public static HardLimits unbounded() {
            return new HardLimits(Rad.of(Double.NEGATIVE_INFINITY), Rad.of(Double.POSITIVE_INFINITY));
        }

        public static final Struct<HardLimits> struct = ProceduralStructGenerator.genRecord(HardLimits.class);
    }

    /**
     * Defines the state of a mechanism.
     */
    public record MechanismState(
        Angle position,
        AngularVelocity velocity,
        AngularAcceleration acceleration
    ) implements StructSerializable {

        public static MechanismState of(Angle angle, AngularVelocity velocity, AngularAcceleration acceleration) {
            return new MechanismState(angle, velocity, acceleration);
        }

        public static MechanismState zero() {
            return new MechanismState(Rad.zero(), RadPS.zero(), RadPS2.zero());
        }

        public MechanismState times(double scalar) {
            return new MechanismState(position.times(scalar), velocity.times(scalar), acceleration.times(scalar));
        }

        public MechanismState div(double scalar) {
            return times(1.0 / scalar);
        }

        public static final Struct<MechanismState> struct = ProceduralStructGenerator.genRecord(MechanismState.class);
    }

    /**
     * Defines some variables of a mechanism in the current time step.
     */
    public record MechanismVariables(
        Torque torque,
        Voltage statorVoltage,
        Voltage supplyVoltage,
        Current statorCurrent
    ) implements StructSerializable {

        public Current supplyCurrent() {
            // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10;
            return statorCurrent.times(statorVoltage.div(supplyVoltage)).div(kMotorEfficiency);
        }

        public static MechanismVariables of(Torque torque, Voltage voltage, Voltage supplyVoltage, Current statorCurrent) {
            return new MechanismVariables(torque, voltage, supplyVoltage, statorCurrent);
        }

        public static MechanismVariables zero() {
            return new MechanismVariables(NewtonMeters.zero(), Volts.zero(), Volts.zero(), Amps.zero());
        }

        public static final Struct<MechanismVariables> struct = ProceduralStructGenerator.genRecord(MechanismVariables.class);
    }

    private final DataLogger logger;
    private final String name;
    private final MechanismDynamics dynamics;
    private final Friction friction;
    private final GearRatio gearRatio;
    private final DCMotor2 motor;
    private final ShamMotorController controller;
    private final ShamEnvTiming timing;
    private final MomentOfInertia rotorInertia;
    private final HardLimits limits;
    private final double noise;

    private final ReentrantReadWriteLock ioLock = new ReentrantReadWriteLock();
    private MechanismState state = MechanismState.zero();
    private MechanismVariables variables = MechanismVariables.zero();

    public ShamMechanism(
        String name,
        DCMotor motor,
        ShamMotorController controller,
        MomentOfInertia rotorInertia,
        GearRatio gearRatio,
        Friction friction,
        MechanismDynamics dynamics,
        HardLimits limits,
        double noise,
        ShamEnvTiming timing
    ) {
        this.logger = RuntimeLog.loggerFor("Mechanism/"+name);
        this.dynamics = dynamics;
        this.friction = friction;
        this.gearRatio = gearRatio;
        this.motor = new DCMotor2(motor);
        this.controller = controller;
        this.timing = timing;
        this.rotorInertia = rotorInertia;
        this.name = name;
        this.limits = limits;
        this.noise = noise;

        logger.log("dynamics", dynamics.getClass().getSimpleName());
        logger.log("friction", friction, Friction.struct);
        logger.log("gearRatio", gearRatio, GearRatio.struct);
        logger.log("motor", motor, DCMotor.struct);
        logger.log("controller", controller.getClass().getSimpleName());
        logger.log("rotorInertia", rotorInertia);
        logger.log("limits", limits, HardLimits.struct);
        logger.log("noise", noise);
    }

    /**
     * Gets the current state of the mechanism.
     * 
     * @return the current state of the mechanism
     */
    public MechanismState state() {
        try {
            ioLock.readLock().lock();
            return state;
        } finally {
            ioLock.readLock().unlock();
        }
    }

    /**
     * Gets the current state of the motor driving the mechanism.
     * 
     * @return the current state of the mechanism
     */
    public MechanismState motorState() {
        return state().times(gearRatio.getReduction());
    }

    public void setState(MechanismState state) {
        try {
            ioLock.writeLock().lock();
            this.state = state;
        } finally {
            ioLock.writeLock().unlock();
        }
    }

    public MechanismVariables variables() {
        try {
            ioLock.readLock().lock();
            return variables;
        } finally {
            ioLock.readLock().unlock();
        }
    }

    public MechanismVariables motorVariables() {
        var v = variables();
        return MechanismVariables.of(
            v.torque.div(gearRatio.getReduction()),
            v.statorVoltage,
            v.supplyVoltage,
            v.statorCurrent
        );
    }

    /**
     * Gets the name of the mechanism.
     * 
     * @return the name of the mechanism
     */
    public String name() {
        return name;
    }

    /**
     * Gets the torque applied by the motor when "braking" is enabled.
     * 
     * <p> This value is before any gear reduction is applied.
     * 
     * @return the torque applied by the motor when "braking" is enabled.
     */
    protected Torque getMotorBrakingTorque() {
        if (!controller.brakeEnabled()) {
            logger.log("Update/braking", false);
            logger.log("Update/brakingTorque", 0.0);
            logger.log("Update/brakingCurrent", 0.0);
            return NewtonMeters.zero();
        }
        Current current = motor.getCurrent(motorState().velocity(), Volts.zero());
        Torque torque = motor.getTorque(current);
        logger.log("Update/braking", true);
        logger.log("Update/brakingTorque", torque);
        logger.log("Update/brakingCurrent", current);
        return torque.times(gearRatio.getReduction());
    }

    /**
     * Applies friction and braking to the output of the mechanism.
     * This will reduce the torque applied to the mechanism.
     * 
     * @param torque the torque to apply to the mechanism
     * @return the torque after friction has been applied
     */
    protected Torque calculateResistanceTorque(Torque motorOutput, Torque environment, MomentOfInertia inertia) {
        AngularVelocity velocity = state().velocity();
        boolean isMoving = MeasureMath.abs(velocity).gt(RadPS.zero());
        logger.log("Update/moving", isMoving);
        Torque frictionTorque = isMoving ? friction.kineticFriction() : friction.staticFriction();
        Torque brakingTorque = NewtonMeters.zero();
        if (MeasureMath.abs(motorOutput).lt(NewtonMeters.of(0.01))) {
            brakingTorque = getMotorBrakingTorque();
        }
        // ensure that friction/braking opposes the motion
        Torque antiTorque = frictionTorque.plus(brakingTorque)
                .times(-MeasureMath.signum(velocity) * gearRatio.getReduction());

        logger.log("Update/frictionTorque", frictionTorque);
        logger.log("Update/desiredAntiTorque", antiTorque);

        final AngularAcceleration unburdenedAccel = div(motorOutput.plus(environment), inertia);
        final AngularAcceleration burdenAccel = div(antiTorque, inertia);
        final AngularAcceleration imposedAccel = unburdenedAccel.plus(burdenAccel);
        logger.log("Update/unburdenedAccel", unburdenedAccel);
        logger.log("Update/burdenAccel", burdenAccel);
        logger.log("Update/imposedAccel", imposedAccel);

        // the goal of anti torque is to reduce the velocity to 0
        // so we need to ensure that the anti torque is not so large that it causes the mechanism to reverse

        final AngularAcceleration accelNeededToStop = velocity.times(-1.0).div(timing.dt());
        final Torque torqueNeededToStop = MeasureMath.times(accelNeededToStop, inertia);
        logger.log("Update/accelNeededToStop", accelNeededToStop);

        if (accelNeededToStop.lt(RadPS2.zero())) {
            // accel needed to stop is negative
            if (imposedAccel.lt(accelNeededToStop)) {
                // the imposed accel has a greater magnitude in the same sign
                // as the accel needed to stop, this will cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", torqueNeededToStop);
                return torqueNeededToStop;
            } else {
                // the imposed accel has a lesser magnitude in the same sign
                // as the accel needed to stop, this will cannot cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", antiTorque);
                return antiTorque;
            }
        } else {
            // accel needed to stop is positive
            if (imposedAccel.gt(accelNeededToStop)) {
                // the imposed accel has a greater magnitude in the same sign
                // as the accel needed to stop, this will cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", torqueNeededToStop);
                return torqueNeededToStop;
            } else {
                // the imposed accel has a lesser magnitude in the same sign
                // as the accel needed to stop, this will cannot cause the mechanism
                // to reverse
                logger.log("Update/antiTorque", antiTorque);
                return antiTorque;
            }
        }
    }

    protected Current getMotorCurrent(Voltage supplyVoltage) {
        ControllerOutput co = controller.run(timing.dt(), supplyVoltage, motorState());
        if (DriverStation.isDisabled()) {
            return Amps.zero();
        }
        if (co instanceof ControllerOutput.VoltageOutput vo) {
            Voltage voltage = vo.voltage();
            if (voltage.isNear(Volts.zero(), Volts.of(0.01))) {
                return Amps.zero();
            }
            return motor.getCurrent(motorState().velocity(), voltage);
        } else if (co instanceof ControllerOutput.CurrentOutput io) {
            return io.current();
        } else {
            return Amps.zero();
        }
    }

    /**
     * Updates the state of the mechanism.
     */
    void update(final Voltage supplyVoltage) {
        final Time dt = timing.dt();

        final MomentOfInertia inertia = rotorInertia.plus(dynamics.extraInertia());
        logger.log("Update/inertia", inertia);

        // calculate the torque acting on the mechanism
        final Current motorCurrent = getMotorCurrent(supplyVoltage);
        final Torque motorTorque = motor.getTorque(motorCurrent);
        final Torque mechanismTorque = motorTorque.times(gearRatio.getReduction());
        final Torque environment = dynamics.environment(state());
        final Torque antiTorque = calculateResistanceTorque(mechanismTorque, environment, inertia);
        final Torque outputTorque = mechanismTorque.plus(antiTorque);
        logger.log("Update/motorCurrent", motorCurrent);
        logger.log("Update/motorTorque", motorTorque);
        logger.log("Update/mechanismTorque", mechanismTorque);
        logger.log("Update/environmentTorque", environment);
        logger.log("Update/antiTorque", antiTorque);
        logger.log("Update/outputTorque", outputTorque);

        // calculate the displacement, velocity, and acceleration of the mechanism
        final AngularAcceleration acceleration = div(outputTorque, inertia)
            .times(1.0 + (noise * RAND.nextGaussian()));
        final AngularVelocity velocity = MeasureMath.nudgeZero(
            state().velocity().plus(acceleration.times(dt)),
            RotationsPerSecond.of(0.001)
        );
        final Angle angle = state().position().plus(velocity.times(dt));

        try {
            ioLock.writeLock().lock();
            // apply hard limits
            // then update the state accordingly
            if (angle.lt(limits.minAngle())) {
                state = MechanismState.of(limits.minAngle(), RadPS.zero(), RadPS2.zero());
            } else if (angle.gt(limits.maxAngle())) {
                state = MechanismState.of(limits.maxAngle(), RadPS.zero(), RadPS2.zero());
            } else {
                state = MechanismState.of(angle, velocity, acceleration);
            }
            // capture the "variables"
            variables = MechanismVariables.of(
                mechanismTorque,
                motor.getVoltage(motorTorque, motorState().velocity()),
                supplyVoltage,
                motorCurrent
            );
        } finally {
            logger.log("Update/state", state, MechanismState.struct);
            logger.log("Update/motorState", motorState(), MechanismState.struct);
            logger.log("Update/variables", variables, MechanismVariables.struct);
            ioLock.writeLock().unlock();
        }
    }
}
