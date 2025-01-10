package igknighters.commands.autos;

import static igknighters.commands.autos.Waypoints.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.subsystems.Subsystems;

public class AutoRoutines extends AutoCommands {

  private final AutoFactory factory;

  public AutoRoutines(Subsystems subsystems, Localizer localizer, AutoFactory factory) {
    super(subsystems, localizer);
    this.factory = factory;

    if (Robot.isSimulation()) {
      new Trigger(DriverStation::isAutonomousEnabled)
          .onTrue(
              Commands.waitSeconds(15.3)
                  .andThen(() -> DriverStationSim.setEnabled(false))
                  .withName("Simulated Auto Ender"));
    }
  }

  public AutoRoutine test() {
    return factory.newRoutine(C1.to(C1));
  }
}
