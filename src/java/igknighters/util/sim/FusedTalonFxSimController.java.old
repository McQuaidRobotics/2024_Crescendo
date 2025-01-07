package igknighters.util.sim;

import java.util.concurrent.atomic.AtomicBoolean;

import sham.ShamMotorController;
import sham.ShamMechanism.MechanismOutputs;
import sham.utils.GearRatio;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class FusedTalonFxSimController implements ShamMotorController {
    private final TalonFXSimState talonSimState;
    private final CANcoderSimState cancoderSimState;
    private final GearRatio rotorToSensor;
    private final AtomicBoolean brakeEnabled = new AtomicBoolean(false);

    public FusedTalonFxSimController(
        TalonFXSimState talonSimState,
        CANcoderSimState cancoderSimState,
        GearRatio rotorToSensor
    ) {
        this.talonSimState = talonSimState;
        this.cancoderSimState = cancoderSimState;
        this.rotorToSensor = rotorToSensor;
    }

    public FusedTalonFxSimController withBrakeEnabled(boolean brakeEnabled) {
        this.brakeEnabled.set(brakeEnabled);
        return this;
    }

    @Override
    public boolean brakeEnabled() {
        return brakeEnabled.get();
    }

    @Override
    public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
        talonSimState.setSupplyVoltage(supply);
        talonSimState.setRawRotorPosition(state.position());
        talonSimState.setRotorVelocity(state.velocity());
        talonSimState.setRotorAcceleration(state.acceleration());

        MechanismOutputs sensorState = state.times(rotorToSensor.getOverdrive());
        cancoderSimState.setSupplyVoltage(supply);
        cancoderSimState.setRawPosition(sensorState.position());
        cancoderSimState.setVelocity(sensorState.velocity());
        cancoderSimState.setMagnetHealth(MagnetHealthValue.Magnet_Green);

        return talonSimState.getMotorVoltageMeasure();
    }
}
