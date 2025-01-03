package sham;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import sham.ShamMechanism.MechanismState;
import sham.ShamMotorController.ControllerOutput.VoltageOutput;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * An interface to represent a motor controller in the simulation.
 * This can be used to allow CTRE/REV sim to work with the sham library.
 */
public interface ShamMotorController {
    public sealed interface ControllerOutput {
        public record VoltageOutput(Voltage voltage) implements ControllerOutput {
            public static VoltageOutput of(double voltage) {
                return new VoltageOutput(Volts.of(voltage));
            }
        }
        public record CurrentOutput(Current current) implements ControllerOutput {
            public static CurrentOutput of(double current) {
                return new CurrentOutput(Amps.of(current));
            }
        }

        public static VoltageOutput of(Voltage voltage) {
            return new VoltageOutput(voltage);
        }

        public static CurrentOutput of(Current current) {
            return new CurrentOutput(current);
        }

        public static ControllerOutput zero() {
            return of(Amps.of(0));
        }

        default public double signumMagnitude() {
            if (this instanceof VoltageOutput) {
                return Math.signum(((VoltageOutput) this).voltage().baseUnitMagnitude());
            } else {
                return Math.signum(((CurrentOutput) this).current().baseUnitMagnitude());
            }
        }
    }

    /**
     * Runs the simulation for the motor.
     * 
     * @param dt the time step, this will not be different between invocations
     *     of this method on the same instance
     * @param supply the supply voltage
     * @param state the current state of the mechanism,
     *    this is the state <b>at the rotor</b>.
     *    If you wish to scale these states based on the gearbox ratio,
     *    you can do `state.times(gearboxRatio)`.
     * @return the output voltage
     */
    ControllerOutput run(Time dt, Voltage supply, MechanismState state);

    /**
     * Returns whether the brake is enabled.
     * 
     * @return whether the brake is enabled
     */
    boolean brakeEnabled();

    /**
     * Returns a motor controller that does nothing.
     * 
     * @return a motor controller that does nothing
     */
    public static ShamMotorController none() {
        return new ShamMotorController() {
            @Override
            public ControllerOutput run(Time dt, Voltage supply, MechanismState state) {
                return new VoltageOutput(Volts.of(0));
            }

            @Override
            public boolean brakeEnabled() {
                return false;
            }
        };
    }
}
