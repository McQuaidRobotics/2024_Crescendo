package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface Camera {

    /**
     * A configuration for a camera.
     * This allows to statically define cameras without instantiating them.
     */
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

    /**
     * Creates a configuration for a camera.
     * @param cameraName The name of the camera
     * @param id The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     * @return The configuration
     */
    public static CameraConfig createConfig(String cameraName, Integer id, Pose3d cameraPose) {
        return new CameraConfig(cameraName, id, cameraPose);
    }

    /**
     * Creates a camera from a configuration.
     * @param config The configuration
     * @return The camera
     */
    public static Camera create(CameraConfig config) {
        if (RobotBase.isSimulation()) {
            return new CameraDisabled(config.cameraName, config.id, config.cameraPose);
        } else {
            return new CameraReal(config.cameraName, config.id, config.cameraPose);
        }
    }

    /**
     * Uses the cameras PoseEstimation pipeline to estimate the pose of the robot.
     * @return An optional containing the pose estimation if it was successful
     */
    public Optional<VisionPoseEst> evalPose();

    /**
     * Gets the transform from the robot to the camera.
     * @return The transform
     * 
     * @apiNote This has to be very accurate, otherwise multi-camera pose estimation will suffer a lot.
     */
    public Transform3d getRobotToCameraTransform3d();

    /**
     * Gets the ID of the camera.
     * @return The ID of the camera
     */
    public Integer getId();

    /**
     * Gets the name of the camera.
     * @return The name of the camera
     */
    public String getName();

    /**
     * A pose estimation from a camera.
     */
    public class VisionPoseEst {
        public final int cameraId;
        public final Pose3d pose;
        /** The timestamp of the measurement in seconds */
        public final double timestamp;
        public final List<Integer> apriltags;

        public VisionPoseEst(int cameraId, Pose3d pose, double timestamp, List<Integer> apriltags) {
            this.cameraId = cameraId;
            this.pose = pose;
            this.timestamp = timestamp;
            this.apriltags = apriltags;
        }
    }
}
