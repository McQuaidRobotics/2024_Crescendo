package com.igknighters.vision.camera;

import java.util.List;
import java.util.function.Function;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

/**
 * An abstraction for a photon camera.
 */
public class CameraReal implements Camera {
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
    public CameraReal(String cameraName, Integer id, Pose3d cameraPose, Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> filter) {
        this.camera = new PhotonCamera(cameraName);
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
    public CameraReal(String camerName, Integer id, Pose3d cameraPose) {
        this(camerName, id, cameraPose, CameraReal::filterTargets);
    }

    private static List<PhotonTrackedTarget> filterTargets(List<PhotonTrackedTarget> targets) {
        return targets.stream().filter(target -> target.getPoseAmbiguity() < 0.2).toList();
    }

    @Override
    public CameraResult getLatestResult() {
        return new CameraResult(camera.getLatestResult(), this);
    }

    @Override
    public Boolean hasCalibrationData() {
        return camera.getCameraMatrix().isPresent() && camera.getDistCoeffs().isPresent();
    }

    @Override
    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        return camera.getCameraMatrix();
    }

    @Override
    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        return camera.getDistCoeffs();
    }

    @Override
    public Transform3d getRobotToCameraTransform3d() {
        return cameraPose;
    }

    @Override
    public Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> getFilter() {
        return filter;
    }

    @Override
    public Integer getId() {
        return id;
    }
}
