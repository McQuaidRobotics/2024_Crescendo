package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface Camera {

    public static class CameraConfig {
        public final String cameraName;
        public final Integer id;
        public final Pose3d cameraPose;

        public CameraConfig(String cameraName, Integer id, Pose3d cameraPose) {
            this.cameraName = cameraName;
            this.id = id;
            this.cameraPose = cameraPose;
        }
    }

    public static CameraConfig createConfig(String cameraName, Integer id, Pose3d cameraPose) {
        return new CameraConfig(cameraName, id, cameraPose);
    }

    public static Camera create(CameraConfig config) {
        if (RobotBase.isSimulation()) {
            return new CameraDisabled(config.cameraName, config.id, config.cameraPose);
        } else {
            return new CameraReal(config.cameraName, config.id, config.cameraPose);
        }
    }

    /**
     * Gets the latest result from the camera.
     * 
     * @return The latest result
     */
    public Optional<VisionPoseEst> evalPose();

    public Transform3d getRobotToCameraTransform3d();

    public Integer getId();

    public String getName();

    public class VisionPoseEst {
        public final int cameraId;
        public final Pose3d pose;
        public final double timestamp;
        public final int[] apriltags;

        public VisionPoseEst(int cameraId, Pose3d pose, double timestamp, int[] apriltags) {
            this.cameraId = cameraId;
            this.pose = pose;
            this.timestamp = timestamp;
            this.apriltags = apriltags;
        }
    }
}
