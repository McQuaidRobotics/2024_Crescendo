package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Transform3d;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import igknighters.util.logging.BootupLogger;
import java.util.List;

@SuppressWarnings("unused")
public class CameraDisabled extends Camera {
  private final Transform3d cameraTransform;
  private final String cameraName;

  public CameraDisabled(String cameraName, Transform3d cameraTransform) {
    this.cameraTransform = cameraTransform;
    this.cameraName = cameraName;

    BootupLogger.bootupLog("    " + cameraName + " camera initialized (disabled)");
  }

  @Override
  public List<VisionUpdate> flushUpdates() {
    return List.of();
  }

  @Override
  public Transform3d getRobotToCameraTransform3d() {
    return cameraTransform;
  }

  @Override
  public String getName() {
    return cameraName;
  }

  @Override
  public List<Integer> getSeenTags() {
    return List.of();
  }

  @Override
  public void periodic() {}
}
