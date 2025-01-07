package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Transform3d;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.vision.Vision.VisionUpdate;
import igknighters.subsystems.vision.Vision.VisionUpdateFaults;
import igknighters.util.logging.BootupLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An abstraction for a photon camera. */
public class CameraRealPhoton extends Camera {
  protected final PhotonCamera camera;
  protected final Transform3d cameraPose;
  private final PhotonPoseEstimator poseEstimator;

  private Optional<VisionUpdate> previousUpdate = Optional.empty();
  private ArrayList<Integer> seenTags = new ArrayList<>();
  private ArrayList<VisionUpdate> updates = new ArrayList<>();

  public CameraRealPhoton(String cameraName, Transform3d cameraPose) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraPose = cameraPose;

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.APRIL_TAG_FIELD,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            this.cameraPose);
    poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);

    BootupLogger.bootupLog("    " + cameraName + " camera initialized (real)");
  }

  private VisionUpdate update(EstimatedRobotPose estRoboPose) {

    seenTags.clear();
    for (PhotonTrackedTarget target : estRoboPose.targetsUsed) {
      seenTags.add(target.fiducialId);
    }

    VisionUpdateFaults faults = VisionUpdateFaults.empty();
    if (previousUpdate.isPresent()) {
      double avgDistance =
          estRoboPose.targetsUsed.stream()
              .map(PhotonTrackedTarget::getBestCameraToTarget)
              .map(Transform3d::getTranslation)
              .map(t3 -> Math.sqrt(t3.getX() * t3.getX() + t3.getY() * t3.getY()))
              .mapToDouble(Double::doubleValue)
              .average()
              .orElseGet(() -> 100.0);

      faults =
          VisionUpdateFaults.solve(
              estRoboPose.estimatedPose,
              previousUpdate.get().pose(),
              estRoboPose.timestampSeconds - previousUpdate.get().timestamp(),
              avgDistance,
              seenTags,
              List.of());
    }

    var u = new VisionUpdate(estRoboPose.estimatedPose, estRoboPose.timestampSeconds, faults);
    previousUpdate = Optional.of(u);

    return u;
  }

  @Override
  public Transform3d getRobotToCameraTransform3d() {
    return cameraPose;
  }

  @Override
  public String getName() {
    return camera.getName();
  }

  @Override
  public List<VisionUpdate> flushUpdates() {
    var u = updates;
    updates = new ArrayList<>();
    return u;
  }

  @Override
  public List<Integer> getSeenTags() {
    return seenTags;
  }

  @Override
  public void periodic() {
    camera.getAllUnreadResults().stream()
        .map(poseEstimator::update)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .map(this::update)
        .forEach(updates::add);

    log("isConnected", camera.isConnected());
  }
}
