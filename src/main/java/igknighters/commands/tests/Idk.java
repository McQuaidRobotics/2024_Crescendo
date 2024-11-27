package igknighters.commands.tests;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import monologue.Monologue;

public class Idk extends Command {
    private final TalonFX talon = new TalonFX(55);
    private final TalonFXSimState simState = talon.getSimState();

    public Idk() {
        if (Robot.isReal()) {
            throw new IllegalStateException("This command is for simulation only");
        }
    }

    @Override
    public void execute() {
        simState.setSupplyVoltage(12.0);
        simState.setRotorVelocity(0.0);
        talon.setControl(new VoltageOut(11.0));
        Monologue.log("test/outputVoltageSim", simState.getMotorVoltage());
        Monologue.log("test/outputVoltage", talon.getMotorVoltage().getValueAsDouble());
        Monologue.log("test/outputCurrentSim", simState.getTorqueCurrent());
        Monologue.log("test/outputCurrent", talon.getTorqueCurrent().getValueAsDouble());
    }
}
