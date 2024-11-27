package org.ironmaple;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Random;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.ironmaple.SimArena.SimulationTiming;
import org.ironmaple.utils.GearRatio;
import org.ironmaple.utils.mathutils.MeasureMath;

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

public class SimMechanism {
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
     * first in the same timeslot but this can not be promised.
     */
    public interface MechanismDynamics {
        Torque enviroment(Angle angle, AngularVelocity velocity);
        MomentOfInertia extraInertia();

        static MechanismDynamics of(Torque enviroment) {
            return new MechanismDynamics() {
                @Override
                public Torque enviroment(Angle angle, AngularVelocity velocity) {
                    return enviroment;
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
     * A record to define the friction of a mechanism.
     * The only way to obtain this value is to empiracally measure it.
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
    public record Friction(Torque staticFriction, Torque kineticFriction) {
        public static Friction of(Torque staticFriction, Torque kineticFriction) {
            return new Friction(staticFriction, kineticFriction);
        }

        public static Friction of(Torque universalFriction) {
            return new Friction(universalFriction, universalFriction);
        }

        public static Friction of(DCMotor motor, Voltage voltage) {
            double current = motor.getCurrent(0.0, voltage.in(Volts));
            double torque = motor.getTorque(current);
            return new Friction(NewtonMeters.of(torque), NewtonMeters.of(torque));
        }

        public static Friction zero() {
            return new Friction(NewtonMeters.zero(), NewtonMeters.zero());
        }
    }

    public record HardLimits(Angle minAngle, Angle maxAngle) {
        public static HardLimits of(Angle minAngle, Angle maxAngle) {
            return new HardLimits(minAngle, maxAngle);
        }

        public static HardLimits unbounded() {
            return new HardLimits(Rad.of(-1000000000.0), Rad.of(1000000000.0));
        }
    }

    /**
     * A record to define the output of a mechanism.
     * 
     * <p> All measures in this record are immutable.
     */
    public record MechanismOutputs(Angle angle, AngularVelocity velocity, AngularAcceleration acceleration) {
        public static MechanismOutputs of(Angle angle, AngularVelocity velocity, AngularAcceleration acceleration) {
            return new MechanismOutputs(angle, velocity, acceleration);
        }

        public static MechanismOutputs zero() {
            return new MechanismOutputs(Rad.zero(), RadPS.zero(), RadPS2.zero());
        }

        private MechanismOutputs times(double scalar) {
            return new MechanismOutputs(angle.times(scalar), velocity.times(scalar), acceleration.times(scalar));
        }
    }

    /**
     * A record to define the inputs of a mechanism.
     * 
     * <p> All measures in this record are immutable.
     */
    public record MechanismInputs(
        Torque torque,
        Voltage statorVoltage,
        Voltage supplyVoltage,
        Current statorCurrent) {

        public Current supplyCurrent() {
            // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/10;
            return statorCurrent.times(statorVoltage.divide(supplyVoltage)).divide(kMotorEfficiency);
        }

        public static MechanismInputs of(Torque torque, Voltage voltage, Voltage supplyVoltage, Current statorCurrent) {
            return new MechanismInputs(torque, voltage, supplyVoltage, statorCurrent);
        }

        public static MechanismInputs zero() {
            return new MechanismInputs(NewtonMeters.zero(), Volts.zero(), Volts.zero(), Amps.zero());
        }
    }

    private final String name;
    private final MechanismDynamics dynamics;
    private final Friction friction;
    private final GearRatio gearRatio;
    private final DCMotor motor;
    private final SimMotorController controller;
    private final SimulationTiming timing;
    private final MomentOfInertia rotorInertia;
    private final HardLimits limits;
    private final double noise;

    private final ReentrantReadWriteLock ioLock = new ReentrantReadWriteLock();
    private MechanismOutputs outputs = MechanismOutputs.zero();
    private MechanismInputs inputs = MechanismInputs.zero();

    public SimMechanism(
        String name,
        DCMotor motor,
        SimMotorController controller,
        MomentOfInertia rotorInertia,
        GearRatio gearRatio,
        Friction friction,
        MechanismDynamics dynamics,
        HardLimits limits,
        double noise,
        SimulationTiming timing
    ) {
        this.dynamics = dynamics;
        this.friction = friction;
        this.gearRatio = gearRatio;
        this.motor = motor;
        this.controller = controller;
        this.timing = timing;
        this.rotorInertia = rotorInertia;
        this.name = name;
        this.limits = limits;
        this.noise = noise;
    }

    protected Torque getBrakingTorque() {
        if (!controller.brakeEnabled()) {
            return NewtonMeters.zero();
        }
        double current = motor.getCurrent(outputs.velocity().in(RadPS), Volts.zero().in(Volts));
        double torque = motor.getTorque(current);
        return NewtonMeters.of(torque);
    }

    /**
     * Applies friction and braking to the output of the mechanism.
     * This will reduce the torque applied to the mechanism.
     * 
     * @param torque the torque to apply to the mechanism
     * @return the torque after friction has been applied
     */
    protected Torque applyAntiTorque(Torque torque) {
        boolean isMoving = MeasureMath.abs(outputs.velocity()).gt(RadPS.zero());
        Torque anitTorque = isMoving ? friction.kineticFriction() : friction.staticFriction();
        if (MeasureMath.abs(torque).lt(NewtonMeters.of(0.01))) {
            anitTorque = anitTorque.plus(getBrakingTorque());
        }
        // ensure that friction opposes the motion
        if (outputs.velocity.gt(RadPS.zero())) {
            return torque.minus(anitTorque);
        } else if (outputs.velocity.lt(RadPS.zero())) {
            return torque.plus(anitTorque);
        } else /* if (!isMoving) */ {
            // conteract torque but don't invoke movement in the opposite direction
            if (anitTorque.gt(MeasureMath.abs(torque))) {
                // friction is greater than the applied torque,
                // so the mechanism should not move
                return torque.times(0.0);
            } else if (torque.gt(NewtonMeters.zero())) {
                return torque.minus(anitTorque);
            } else {
                return torque.plus(anitTorque);
            }
        }
    }

    /**
     * Gets the motors output voltage based on the current state of the mechanism.
     * 
     * @return the voltage to apply to the motor
     */
    protected Voltage getControllerVoltage(Voltage supplyVoltage) {
        MechanismOutputs outputs = controller.sensorBeforeGearbox() ? outputs().times(gearRatio.getReduction()) : outputs();
        return controller.run(Seconds.of(timing.dt), supplyVoltage, outputs);
    }

    protected Torque getMotorTorque(Voltage voltage) {
        // TODO: check if getMotorTorque should be passed the current velocity or 0
        double current = motor.getCurrent(outputs().velocity().in(RadPS), voltage.in(Volts));
        double torque = motor.getTorque(current);
        return NewtonMeters.of(torque);
    }

    /**
     * Gets the current state of the mechanism.
     * 
     * @return the current state of the mechanism
     */
    public MechanismOutputs outputs() {
        try {
            ioLock.readLock().lock();
            return outputs;
        } finally {
            ioLock.readLock().unlock();
        }
    }

    /**
     * Gets the current outputs of the mechanism.
     * 
     * @return the current outputs of the mechanism
     */
    public MechanismInputs inputs() {
        try {
            ioLock.readLock().lock();
            return inputs;
        } finally {
            ioLock.readLock().unlock();
        }
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
     * Updates the state of the mechanism.
     */
    void update(final Voltage supplyVoltage) {
        final Time dt = Seconds.of(timing.dt);

        // run the motor controller loop
        final Voltage voltage = getControllerVoltage(supplyVoltage);

        // calculate the torque acting on the mechanism
        final Torque enviroment = dynamics.enviroment(outputs().angle(), outputs().velocity());
        final Torque motorTorque = getMotorTorque(voltage).times(gearRatio.getReduction());
        final Torque outputTorque = applyAntiTorque(enviroment.plus(motorTorque));

        // calculate the displacement, velocity, and acceleration of the mechanism
        // https://openstax.org/books/university-physics-volume-1/pages/10-7-newtons-second-law-for-rotation
        final AngularAcceleration acceleration = RadPS2.of(
            outputTorque.in(NewtonMeters)
            / rotorInertia.plus(dynamics.extraInertia()).in(KilogramSquareMeters)
        ).times(1.0 + (noise * RAND.nextGaussian()));
        final AngularVelocity velocity = outputs().velocity().plus(acceleration.times(dt));
        final Angle angle = outputs().angle().plus(velocity.times(dt));

        try {
            ioLock.writeLock().lock();
            // apply hard limits
            // then update the state accordingly
            if (angle.lt(limits.minAngle())) {
                outputs = MechanismOutputs.of(limits.minAngle(), RadPS.zero(), RadPS2.zero());
            } else if (angle.gt(limits.maxAngle())) {
                outputs = MechanismOutputs.of(limits.maxAngle(), RadPS.zero(), RadPS2.zero());
            } else {
                outputs = MechanismOutputs.of(angle, velocity, acceleration);
            }
            // capture the "inputs"
            inputs = MechanismInputs.of(
                motorTorque, voltage, supplyVoltage,
                Amps.of(motor.getCurrent(outputs.velocity.in(RadPS), voltage.in(Volts)))
            );
        } finally {
            ioLock.writeLock().unlock();
        }
    }
}
