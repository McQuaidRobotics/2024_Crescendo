package com.igknighters.vision.camera;

import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.List;
import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

public interface Camera {

    public static Camera create(String cameraName, Integer id, Pose3d cameraPose, Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> filter) {
         if (RobotBase.isSimulation()) {
             return new CameraDisabled(cameraName, id, cameraPose);
         } else {
            return new CameraReal(cameraName, id, cameraPose, filter);
         }
    }

    public static Camera create(String cameraName, Integer id, Pose3d cameraPose) {
         if (RobotBase.isSimulation()) {
             return new CameraDisabled(cameraName, id, cameraPose);
         } else {
            return new CameraReal(cameraName, id, cameraPose);
         }
    }

    /**
     * Gets the latest result from the camera.
     * @return The latest result
     */
    public CameraResult getLatestResult();

    /**
     * If the camera has calibration data.
     * @return If the camera has calibration data
     */
    public default Boolean hasCalibrationData() {
        return this.getCameraMatrix().isPresent() && this.getDistCoeffs().isPresent();
    }

    public Optional<Matrix<N3, N3>> getCameraMatrix();

    public Optional<Matrix<N5, N1>> getDistCoeffs();

    public Transform3d getRobotToCameraTransform3d();

    public Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> getFilter();

    public Integer getId();

    public class CameraResult {
        private final Camera camera;
        private final boolean isValid;
        private final List<PhotonTrackedTarget> targets;
        private final double timestamp;

        CameraResult(PhotonPipelineResult result, Camera camera) {
            boolean valid = result.hasTargets();
            if (valid) {
                valid = result.getTimestampSeconds() > 0;
            }
            this.isValid = valid;
            this.targets = camera.getFilter().apply(result.getTargets());
            this.timestamp = result.getTimestampSeconds();
            this.camera = camera;
        }

        CameraResult(Boolean isValid, List<PhotonTrackedTarget> targets, double timestamp, Camera camera) {
            this.isValid = isValid;
            this.targets = targets;
            this.timestamp = timestamp;
            this.camera = camera;
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
            return camera.getRobotToCameraTransform3d();
        }

        public Integer getId() {
            return camera.getId();
        }

        public double getTimestamp() {
            return timestamp;
        }

        public CameraResult filterAmbiguity(Double maxAmbiguity) {
            return new CameraResult(
                isValid,
                targets.stream().filter((target) -> target.getPoseAmbiguity() < maxAmbiguity).toList(),
                timestamp,
                camera
            );
        }

        public Camera getCamera() {
            return camera;
        }
    }
}
