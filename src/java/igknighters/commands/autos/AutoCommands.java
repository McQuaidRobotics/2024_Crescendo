package igknighters.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.subsystems.Subsystems;
import monologue.Monologue;

public class AutoCommands {
  protected final Subsystems subsystems;
  protected final Localizer localizer;

  protected AutoCommands(Subsystems subsystems, Localizer localizer) {
    this.subsystems = subsystems;
    this.localizer = localizer;
  }

  protected void logAutoEvent(String name, String event) {
    String msg = "[Auto] Command " + name + " " + event;
    if (Robot.isDebug()) System.out.println(msg);
    Monologue.log("AutoEvent", msg);
  }

  protected Command loggedCmd(Command command) {
    return new WrapperCommand(command) {
      @Override
      public void initialize() {
        logAutoEvent(this.getName(), "started");
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
        logAutoEvent(this.getName(), "ended");
      }
    };
  }
}
