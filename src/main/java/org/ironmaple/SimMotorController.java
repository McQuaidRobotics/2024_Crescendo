package org.ironmaple;

import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.SimMechanism.MechanismOutputs;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public interface SimMotorController {
    /**
     * Runs the simulation for the motor.
     * 
     * @param dt the time step, this will not be different inbetween invocations
     *     of this method on the same instance
     * @param supply the supply voltage
     * @param state the current state of the mechanism
     * @return the output voltage
     */
    Voltage run(Time dt, Voltage supply, MechanismOutputs state);

    /**
     * Tells the mechanism if the sensor is before the gearbox
     * or after the gearbox.
     * @return true if the sensor is before the gearbox, false otherwise
     */
    boolean sensorBeforeGearbox();

    boolean brakeEnabled();

    static SimMotorController none() {
        return new SimMotorController() {
            @Override
            public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
                return Volts.of(0);
            }

            @Override
            public boolean sensorBeforeGearbox() {
                return true;
            }

            @Override
            public boolean brakeEnabled() {
                return false;
            }
        };
    }
}
