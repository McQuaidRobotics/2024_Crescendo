package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.igknighters.util.BootupLogger;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

@SuppressWarnings("unused")
public class CameraDisabled implements Camera {
    private final Integer id;
    private final Transform3d cameraPose;
    private final String cameraName;

    private final CameraInput cameraInput;

    public CameraDisabled(String cameraName, Integer id, Transform3d cameraPose) {
        this.id = id;
        this.cameraPose = cameraPose;
        this.cameraName = cameraName;

        cameraInput = new CameraInput(VisionPoseEstimate.empty(id));

        cameraInput.update(
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
    public Optional<VisionPoseEstimate> evalPose() {
        return cameraInput.getLatestPoseEst();
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
    public void periodic() {
        Logger.processInputs("Vision/Camera[" + getName() + "]", cameraInput);
    }
}
