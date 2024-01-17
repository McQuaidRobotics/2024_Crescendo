package com.igknighters.subsystems.vision.camera;

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
    private final Integer id;
    private final Transform3d cameraPose;
    private final String cameraName;

    public CameraDisabled(String cameraName, Integer id, Pose3d cameraPose) {
        this.id = id;
        this.cameraPose = new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());
        this.cameraName = cameraName;
    }

    @Override
    public Optional<VisionPoseEst> evalPose() {
        return Optional.empty();
    }

    @Override
    public Transform3d getRobotToCameraTransform3d() {
        return cameraPose;
    }

    @Override
    public Integer getId() {
        return id;
    }

    @Override
    public String getName() {
        return cameraName;
    }

    @Override
    public void periodic() {}
}
