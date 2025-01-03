package igknighters.util.sim;

import java.util.concurrent.atomic.AtomicBoolean;

import sham.ShamMotorController;
import sham.ShamMechanism.MechanismOutputs;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class TalonFxSimController implements ShamMotorController {
    private final TalonFXSimState simState;
    private final AtomicBoolean brakeEnabled = new AtomicBoolean(false);

    public TalonFxSimController(TalonFXSimState simState) {
        this.simState = simState;
    }

    public TalonFxSimController withBrakeEnabled(boolean brakeEnabled) {
        this.brakeEnabled.set(brakeEnabled);
        return this;
    }

    @Override
    public boolean brakeEnabled() {
        return brakeEnabled.get();
    }

    @Override
    public Voltage run(Time dt, Voltage supply, MechanismOutputs state) {
        simState.setSupplyVoltage(supply);
        simState.setRawRotorPosition(state.position());
        simState.setRotorVelocity(state.velocity());
        simState.setRotorAcceleration(state.acceleration());

        return simState.getMotorVoltageMeasure();
        // return edu.wpi.first.units.Units.Volts.of(3.0);
    }
}
