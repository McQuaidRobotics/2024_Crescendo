package igknighters.commands.tests;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import java.util.HashMap;
import java.util.Map;
import monologue.Monologue;

/** A test chooser that allows for the selection of test routines at runtime. */
public class TestManager {

  private final String path = "/Choosers/TestChooser";
  private final String NONE_NAME = "Nothing";

  private final HashMap<String, Command> autoRoutines =
      new HashMap<>(Map.of(NONE_NAME, Commands.none().withName(NONE_NAME)));

  private final StringEntry selected =
      NetworkTableInstance.getDefault()
          .getTable(path)
          .getStringTopic("selected")
          .getEntry(NONE_NAME);

  private String lastTestRoutineName = NONE_NAME;
  private Command lastTestRoutine = Commands.none().withName(NONE_NAME);

  public TestManager() {
    Monologue.log(path + "/.type", "String Chooser");
    Monologue.log(path + "/default", NONE_NAME);
    Monologue.log(path + "/active", NONE_NAME);
    Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));
  }

  public void update() {
    if ((DriverStation.isDisabled() || Robot.isSimulation())
        && DriverStation.isDSAttached()
        && !Robot.isUnitTest()) {
      if (selected.get().equals(lastTestRoutineName)) return;
      lastTestRoutineName = selected.get();
      lastTestRoutine = autoRoutines.get(lastTestRoutineName);
      Monologue.log(path + "/active", lastTestRoutineName);
    }
  }

  public void addTestRoutine(String name, Command cmd) {
    autoRoutines.put(name, cmd);
    Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));
  }

  public void choose(String choice) {
    if (Robot.isUnitTest()) {
      lastTestRoutineName = choice;
      lastTestRoutine = autoRoutines.get(choice);
    } else {
      selected.set(choice);
      update();
    }
  }

  public Command getSelectedTestRoutine() {
    return lastTestRoutine;
  }
}
