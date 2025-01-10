package igknighters.constants;

import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.util.logging.BootupLogger;
import java.util.Map;

public class RobotConfig {

  public enum RobotID {
    CRASH,
    BURN,
    Unlabeled;

    public final String name;

    RobotID(String name) {
      this.name = name;
    }

    RobotID() {
      this.name = this.name();
    }
  }

  /** If there are duplicate serial entries the tests will fail!!!! */
  private static final Map<String, RobotID> serialToID =
      Map.of(
          "0306adcf", RobotID.CRASH,
          "0306adf3", RobotID.CRASH,
          "ffffffff", RobotID.CRASH,
          "aaaaaaaa", RobotID.CRASH,
          "03260af0", RobotID.CRASH,
          "03260abb", RobotID.CRASH,
          "0306adb6", RobotID.CRASH,
          "032b4b20", RobotID.CRASH);

  private static RobotID currentID = RobotID.Unlabeled;

  public static RobotID getRobotID() {
    if (currentID == RobotID.Unlabeled) {
      String currentSerialNum;
      if (Robot.isReal()) {
        currentSerialNum = RobotController.getSerialNumber();
        if (currentSerialNum == null) {
          throw new RuntimeException("Tried loading robot ID before advantage-kit was ready!");
        }
        currentSerialNum = currentSerialNum.toLowerCase();
      } else {
        currentSerialNum = "ffffffff";
      }
      if (serialToID.containsKey(currentSerialNum)) {
        currentID = serialToID.get(currentSerialNum);
      } else {
        throw new RuntimeException(
            "Robot ID not found, " + currentSerialNum + " not in serialToID map");
      }
      BootupLogger.bootupLog("Robot Name: " + currentID.name);
    }
    return currentID;
  }
}
