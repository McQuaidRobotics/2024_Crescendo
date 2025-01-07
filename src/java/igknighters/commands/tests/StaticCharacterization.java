package igknighters.commands.tests;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class StaticCharacterization extends Command {
  private static final TunableDouble currentRampFactor =
      TunableValues.getDouble("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final TunableDouble minVelocity =
      TunableValues.getDouble("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  public StaticCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRampFactor.value();
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.value();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + currentInput);
    inputConsumer.accept(0);
  }
}
