package igknighters;

import choreo.Choreo;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;

public final class Main {
  static {
    Choreo.setChoreoDir(
        new File(
            Filesystem.getOperatingDirectory(),
            "src" + File.separator + "deploy" + File.separator + "choreo"));
  }

  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
