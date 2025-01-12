package monologue;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class GlobalField {
  private final static Field2d field = new Field2d();

  public static synchronized void setObject(String name, Pose2d pose) {
    field.getObject(name).setPose(pose);
  }

  public static synchronized void setObject(String name, Pose2d... pose) {
    field.getObject(name).setPoses(pose);
  }

  public static synchronized void setObject(String name, List<Pose2d> pose) {
    field.getObject(name).setPoses(pose);
  }
}
