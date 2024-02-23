package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface Camera {
    public static class CameraInput implements LoggableInputs {
        private VisionPoseEstimate latestPoseEst;
        private boolean isPresent = false;

        public CameraInput(VisionPoseEstimate pose) {
            this.latestPoseEst = pose;
        }

        public void update(Optional<VisionPoseEstimate> pose) {
            if (pose.isPresent()) {
                latestPoseEst = pose.get();
                isPresent = true;
            } else {
                isPresent = false;
            }
        }

        public Optional<VisionPoseEstimate> getLatestPoseEst() {
            if (isPresent) {
                return Optional.of(latestPoseEst);
            } else {
                return Optional.empty();
            }
        }

        @Override
        public void toLog(LogTable table) {
            table.put("isPresent", isPresent);
            if (!isPresent) {
                return;
            }
            table.put("latestPoseEst/timestamp", latestPoseEst.timestamp);
            table.put("latestPoseEst/pose", latestPoseEst.pose);
            table.put("latestPoseEst/tags", latestPoseEst.apriltags.stream().mapToInt(i -> i).toArray());
            table.put("latestPoseEst/ambiguity", latestPoseEst.ambiguity);
            table.put("latestPoseEst/maxDistance", latestPoseEst.maxDistance);
        }

        @Override
        public void fromLog(LogTable table) {
            isPresent = table.get("isPresent", isPresent);

            Pose3d pose = table.get("latestPoseEst/pose", latestPoseEst.pose);
            double timestamp = table.get("latestPoseEst/timestamp", latestPoseEst.timestamp);
            int[] tagsPrim = table.get("latestPoseEst/tags",
                    latestPoseEst.apriltags.stream().mapToInt(i -> i).toArray());
            double ambiguity = table.get("latestPoseEst/ambiguity", latestPoseEst.ambiguity);
            double maxDistance = table.get("latestPoseEst/maxDistance", latestPoseEst.maxDistance);

            ArrayList<Integer> tags = new ArrayList<>();
            for (int tag : tagsPrim) {
                tags.add(tag);
            }

            latestPoseEst = new VisionPoseEstimate(
                    latestPoseEst.cameraId,
                    pose,
                    timestamp,
                    tags,
                    ambiguity,
                    maxDistance
                );
        }
    }

    /**
     * A configuration for a camera.
     * This allows to statically define cameras without instantiating them.
     */
    public static class CameraConfig {
        public final String cameraName;
        public final Integer id;
        public final Transform3d cameraPose;

        public CameraConfig(String cameraName, Integer id, Transform3d cameraPose) {
            this.cameraName = cameraName;
            this.id = id;
            this.cameraPose = cameraPose;
        }
    }

    /**
     * Creates a configuration for a camera.
     * 
     * @param cameraName The name of the camera
     * @param id         The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     * @return The configuration
     */
    public static CameraConfig createConfig(String cameraName, Integer id, Transform3d cameraPose) {
        return new CameraConfig(cameraName, id, cameraPose);
    }

    /**
     * Creates a camera from a configuration.
     * 
     * @param config The configuration
     * @return The camera
     */
    public static Camera create(CameraConfig config) {
        if (RobotBase.isSimulation()) {
            return new CameraDisabled(config.cameraName, config.id, config.cameraPose);
        } else {
            try {
                return new CameraReal(config.cameraName, config.id, config.cameraPose);
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), e.getStackTrace());
                return new CameraDisabled(config.cameraName, config.id, config.cameraPose);
            }
        }
    }

    /**
     * Uses the cameras PoseEstimation pipeline to estimate the pose of the robot.
     * 
     * @return An optional containing the pose estimation if it was successful
     */
    public Optional<VisionPoseEstimate> evalPose();

    /**
     * Gets the transform from the robot to the camera.
     * 
     * @return The transform
     * 
     * @apiNote This has to be very accurate, otherwise multi-camera pose estimation
     *          will suffer a lot.
     */
    public Transform3d getRobotToCameraTransform3d();

    /**
     * Gets the ID of the camera.
     * 
     * @return The ID of the camera
     */
    public Integer getId();

    /**
     * Gets the name of the camera.
     * 
     * @return The name of the camera
     */
    public String getName();

    public void periodic();

    /**
     * A pose estimation from a camera.
     */
    public class VisionPoseEstimate {
        public final int cameraId;
        public final Pose3d pose;
        /** The timestamp of the measurement in seconds */
        public final double timestamp;
        public final List<Integer> apriltags;
        public final double ambiguity;
        public final double maxDistance;

        public VisionPoseEstimate(int cameraId, Pose3d pose, double timestamp, List<Integer> apriltags,
                double ambiguity, double maxDistance) {
            this.cameraId = cameraId;
            this.pose = pose;
            this.timestamp = timestamp;
            this.apriltags = apriltags;
            this.ambiguity = ambiguity;
            this.maxDistance = maxDistance;
        }
    }
}
