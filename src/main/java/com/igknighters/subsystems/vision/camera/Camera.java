package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kVision;

public interface Camera {
    public static class CameraInput implements LoggableInputs {
        private VisionPoseEstimate latestPoseEst;
        private VisionEstimateFault latestFault;
        private boolean isPresent = false, isConnected = false;

        public CameraInput(VisionPoseEstimate pose) {
            this.latestPoseEst = pose;
        }

        public void update(Optional<Pair<VisionPoseEstimate, VisionEstimateFault>> estimate, boolean isConnected) {
            if (estimate.isPresent()) {
                latestPoseEst = estimate.get().getFirst();
                latestFault = estimate.get().getSecond();
                isPresent = true;
            } else {
                isPresent = false;
            }
            this.isConnected = isConnected;
        }

        public Optional<VisionPoseEstimate> getLatestPoseEst() {
            if (isPresent) {
                return Optional.of(latestPoseEst);
            } else {
                return Optional.empty();
            }
        }

        public VisionEstimateFault getLatestFault() {
            if (isPresent) {
                return latestFault;
            } else {
                return new VisionEstimateFault(false, false, false, false, false, false, false, false, false);
            }
        }

        @Override
        public void toLog(LogTable table) {
            table.put("isConnected", isConnected);
            table.put("isPresent", isPresent);
            if (!isPresent) {
                return;
            }
            latestPoseEst.toLog(table);
            latestFault.toLog(table);
        }

        @Override
        public void fromLog(LogTable table) {
            isConnected = table.get("isConnected", isConnected);
            isPresent = table.get("isPresent", isPresent);

            if (!isPresent) {
                return;
            }

            latestPoseEst = VisionPoseEstimate.fromLog(table);
            latestFault = VisionEstimateFault.fromLog(table);
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
     * Gets the faults of the last pose estimation.
     * 
     * @return The faults
     */
    public VisionEstimateFault getFaults();

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

    public record VisionPoseEstimate(
            int cameraId,
            Pose3d pose,
            double timestamp,
            List<Integer> apriltags,
            double ambiguity,
            double maxDistance) {
        public double distanceFrom(VisionPoseEstimate other) {
            return pose.getTranslation().getDistance(other.pose.getTranslation());
        }

        public void toLog(LogTable table) {
            table.put("cameraId", cameraId);
            table.put("pose", pose);
            table.put("timestamp", timestamp);
            table.put("apriltags", apriltags.stream().mapToInt(i -> i).toArray());
            table.put("ambiguity", ambiguity);
            table.put("maxDistance", maxDistance);
        }

        public static VisionPoseEstimate fromLog(LogTable table) {
            int cameraId = table.get("cameraId", 0);
            Pose3d pose = table.get("pose", new Pose3d());
            double timestamp = table.get("timestamp", 0.0);
            int[] tagsPrim = table.get("apriltags", new int[0]);
            double ambiguity = table.get("ambiguity", 0.0);
            double maxDistance = table.get("maxDistance", 0.0);

            ArrayList<Integer> tags = new ArrayList<>();
            for (int tag : tagsPrim) {
                tags.add(tag);
            }

            return new VisionPoseEstimate(cameraId, pose, timestamp, tags, ambiguity, maxDistance);
        }

        public Pair<VisionPoseEstimate, VisionEstimateFault> withFault(
                VisionPoseEstimate last,
                Timer jitterTimer,
                Consumer<VisionPoseEstimate> jitterReseter) {
            Translation2d simplePose = pose.getTranslation().toTranslation2d();
            boolean oob = simplePose.getX() < 0.0
                    || simplePose.getX() > FieldConstants.FIELD_LENGTH
                    || simplePose.getY() < 0.0
                    || simplePose.getY() > FieldConstants.FIELD_WIDTH;
            VisionEstimateFault fault = new VisionEstimateFault(
                    oob,
                    maxDistance > 4.5,
                    ambiguity > kVision.AMBIGUITY_CUTOFF,
                    this.distanceFrom(last) > jitterTimer.get() * kSwerve.MAX_DRIVE_VELOCITY,
                    apriltags.isEmpty(),
                    Math.abs(pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA,
                    pose.getRotation().getY() > kVision.MAX_ANGLE_DELTA,
                    pose.getRotation().getX() > kVision.MAX_ANGLE_DELTA,
                    false);

            if (!fault.extremeJitter) {
                jitterReseter.accept(this);
            }

            return new Pair<>(this, fault);
        }
    }

    public record VisionEstimateFault(
            boolean outOfBounds,
            boolean outOfRange,
            boolean tooAmbiguous,
            boolean extremeJitter,
            boolean noTags,
            boolean infeasibleZValue,
            boolean infeasiblePitchValue,
            boolean infeasibleRollValue,
            boolean isDisabled) {
        public void toLog(LogTable table) {
            final String p = "VisionEstimateFault/";
            table.put(p + "outOfBounds", outOfBounds);
            table.put(p + "outOfRange", outOfRange);
            table.put(p + "tooAmbiguous", tooAmbiguous);
            table.put(p + "extremeJitter", extremeJitter);
            table.put(p + "noTags", noTags);
            table.put(p + "infeasibleZValue", infeasibleZValue);
            table.put(p + "infeasiblePitchValue", infeasiblePitchValue);
            table.put(p + "infeasibleRollValue", infeasibleRollValue);
            table.put(p + "isDisabled", isDisabled);
        }

        public static VisionEstimateFault fromLog(LogTable table) {
            final String p = "VisionEstimateFault/";
            return new VisionEstimateFault(
                    table.get(p + "outOfBounds", false),
                    table.get(p + "outOfRange", false),
                    table.get(p + "tooAmbiguous", false),
                    table.get(p + "extremeJitter", false),
                    table.get(p + "noTags", false),
                    table.get(p + "infeasibleZValue", false),
                    table.get(p + "infeasiblePitchValue", false),
                    table.get(p + "infeasibleRollValue", false),
                    table.get(p + "isDisabled", false));
        }

        public boolean isFaulty() {
            return outOfBounds || outOfRange || tooAmbiguous || extremeJitter || noTags || infeasibleZValue
                    || infeasiblePitchValue || infeasibleRollValue || isDisabled;
        }
    }
}
