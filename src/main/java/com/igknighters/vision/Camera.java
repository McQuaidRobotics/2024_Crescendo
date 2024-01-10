package com.igknighters.vision;

import java.util.List;
import java.util.function.Function;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * An abstraction for a photon camera.
 */
public class Camera {
    private final PhotonCamera camera;
    private final Integer id;
    private final Transform3d cameraPose;
    private Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> filter;

    /**
     * Creates an abstraction for a photon camera.
     * @param cameraName The name of the camera
     * @param id The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     * @param filter The filter to apply to the targets
     */
    public Camera(String cameraName, Integer id, Pose3d cameraPose, Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> filter) {
        if (RobotBase.isReal()) {
            this.camera = new PhotonCamera(cameraName);
        } else {
            this.camera = null;
        }
        this.id = id;
        this.cameraPose = new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
        this.filter = filter;
    }

    /**
     * Creates an abstraction for a photon camera.
     * @param camerName The name of the camera
     * @param id The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     */
    public Camera(String camerName, Integer id, Pose3d cameraPose) {
        this(camerName, id, cameraPose, Camera::filterTargets);
    }

    private static List<PhotonTrackedTarget> filterTargets(List<PhotonTrackedTarget> targets) {
        return targets.stream().filter(target -> target.getPoseAmbiguity() < 0.2).toList();
    }

    /**
     * Gets the latest result from the camera.
     * @return The latest result
     */
    public CameraResult getLatestResult() {
        return new CameraResult(camera.getLatestResult());
    }

    /**
     * If the camera has calibration data.
     * @return If the camera has calibration data
     */
    public Boolean hasCalibrationData() {
        return camera.getCameraMatrix().isPresent() && camera.getDistCoeffs().isPresent();
    }

    /**
     * Gets the underlying photon camera for advanced usage.
     * @return The underlying photon camera
     */
    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    public Transform3d getRobotToCameraTransform3d() {
        return cameraPose;
    }

    public class CameraResult {
        private final boolean isValid;
        private final List<PhotonTrackedTarget> targets;
        private final double timestamp;

        private CameraResult(PhotonPipelineResult result) {
            boolean valid = result.hasTargets();
            if (valid) {
                valid = result.getTimestampSeconds() > 0;
            }
            this.isValid = valid;
            this.targets = filter.apply(result.getTargets());
            this.timestamp = result.getTimestampSeconds();
        }

        private CameraResult(Boolean isValid, List<PhotonTrackedTarget> targets, double timestamp) {
            this.isValid = isValid;
            this.targets = targets;
            this.timestamp = timestamp;
        }

        public Boolean isValid() {
            return isValid;
        }

        public Boolean multiTarget() {
            return targets.size() > 1;
        }

        public List<PhotonTrackedTarget> getTargets() {
            return targets;
        }

        public Transform3d getCameraTransform3d() {
            return cameraPose;
        }

        public Integer getId() {
            return id;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public CameraResult filterAmbiguity(Double maxAmbiguity) {
            return new CameraResult(
                isValid,
                targets.stream().filter((target) -> target.getPoseAmbiguity() < maxAmbiguity).toList(),
                timestamp
            );
        }

        public Camera getCamera() {
            return Camera.this;
        }
    }
}
