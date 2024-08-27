package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.igknighters.util.logging.BootupLogger;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

@SuppressWarnings("unused")
public class CameraDisabled extends Camera {
    private final Integer id;
    private final Transform3d cameraPose;
    private final String cameraName;

    public CameraDisabled(String cameraName, Integer id, Transform3d cameraPose) {
        super(id);
        this.id = id;
        this.cameraPose = cameraPose;
        this.cameraName = cameraName;

        this.update(
            Optional.of(
                Pair.of(
                    VisionPoseEstimate.empty(id),
                    getFaults()
                )
            ),
            false
        );

        BootupLogger.bootupLog("    " + cameraName + " camera initialized (disabled)");
    }

    @Override
    public VisionEstimateFault getFaults() {
        return new VisionEstimateFault(false, false, false, false, false, false, false, false, true);
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
