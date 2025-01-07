package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Transform3d;
import igknighters.subsystems.Component;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import java.util.List;

public abstract class Camera extends Component {

  /**
   * A configuration for a camera. This allows to statically define cameras without instantiating
   * them.
   */
  public record CameraConfig(String cameraName, Transform3d cameraPose) {}

  /**
   * Uses the cameras PoseEstimation pipeline to estimate the pose of the robot.
   *
   * @return A list containing all updates since the last call to this method
   */
  public abstract List<VisionUpdate> flushUpdates();

  /**
   * Gets the transform from the robot to the camera.
   *
   * @return The transform
   * @apiNote This has to be very accurate, otherwise multi-camera pose estimation will suffer a
   *     lot.
   */
  public abstract Transform3d getRobotToCameraTransform3d();

  /**
   * Gets the name of the camera.
   *
   * @return The name of the camera
   */
  public abstract String getName();

  /**
   * Gets the last seen tags by the camera.
   *
   * @return The last seen tags
   */
  public abstract List<Integer> getSeenTags();
}
