package com.igknighters.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

@SuppressWarnings("unused")
public class CameraDisabled implements Camera {
    private final Integer ID;
    private final Transform3d CAMERA_POSE;

    public CameraDisabled(String cameraName, Integer id, Pose3d cameraPose) {
        ID = id;
        CAMERA_POSE = new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
    }

    @Override
    public CameraResult getLatestResult() {
        return null;
    }

    @Override
    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        return Optional.empty();
    }

    @Override
    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        return Optional.empty();
    }

    @Override
    public Transform3d getRobotToCameraTransform3d() {
        return CAMERA_POSE;
    }

    @Override
    public Function<List<PhotonTrackedTarget>, List<PhotonTrackedTarget>> getFilter() {
        return Function.identity();
    }

    @Override
    public Integer getId() {
        return ID;
    }
}
