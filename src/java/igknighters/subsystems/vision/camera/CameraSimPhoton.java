package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import igknighters.SimCtx;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("unused")
public class CameraSimPhoton extends CameraRealPhoton {
  public CameraSimPhoton(String cameraName, Transform3d cameraTransform, SimCtx simCtx) {
    super(cameraName, cameraTransform);

    final SimCameraProperties props = new SimCameraProperties();
    props.setCalibError(0.1, 0.0);
    props.setFPS(43.0);
    props.setCalibration(1280, 800, Rotation2d.fromDegrees(72.0));
    props.setAvgLatencyMs(20.0);
    props.setLatencyStdDevMs(2.0);
    props.setExposureTimeMs(0);
    final PhotonCameraSim sim = new PhotonCameraSim(camera, props, 0.08, 6.5);
    simCtx.aprilTagSim().addCamera(sim, cameraTransform);
  }
}
