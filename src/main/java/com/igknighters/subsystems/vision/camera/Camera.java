package com.igknighters.subsystems.vision.camera;

import edu.wpi.first.wpilibj.DriverStation;
import com.igknighters.Robot;
import edu.wpi.first.wpilibj.Timer;
import monologue.Annotations.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.nio.ByteBuffer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

// import com.igknighters.constants.AprilTags;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.Component;

public abstract class Camera extends Component {
    @Log
    protected VisionPoseEstimate latestPoseEst;
    @Log
    protected VisionEstimateFault latestFault;
    @Log
    protected boolean isPresent = false;
    @Log
    protected boolean isConnected = false;

    protected Camera(int id) {
        this.latestPoseEst = VisionPoseEstimate.empty(id);
        this.latestFault = VisionEstimateFault.empty();
    }

    protected void update(Optional<Pair<VisionPoseEstimate, VisionEstimateFault>> estimate, boolean isConnected) {
        if (estimate.isPresent()) {
            latestPoseEst = estimate.get().getFirst();
            latestFault = estimate.get().getSecond();
            isPresent = true;
        } else {
            isPresent = false;
        }
        this.isConnected = isConnected;
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
        if (Robot.isSimulation()) {
            return new CameraDisabled(config.cameraName, config.id, config.cameraPose);
        } else {
            try {
                return new CameraRealPhoton(config.cameraName, config.id, config.cameraPose);
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
    public Optional<VisionPoseEstimate> evalPose() {
        if (isPresent) {
            return Optional.of(latestPoseEst);
        } else {
            return Optional.empty();
        }
    }

    /**
     * Gets the faults of the last pose estimation.
     * 
     * @return The faults
     */
    public VisionEstimateFault getFaults() {
        if (isPresent) {
            return latestFault;
        } else {
            return new VisionEstimateFault(false, false, false, false, false, false, false, false, false);
        }
    }

    /**
     * Gets the transform from the robot to the camera.
     * 
     * @return The transform
     * 
     * @apiNote This has to be very accurate, otherwise multi-camera pose estimation
     *          will suffer a lot.
     */
    public abstract Transform3d getRobotToCameraTransform3d();

    /**
     * Gets the ID of the camera.
     * 
     * @return The ID of the camera
     */
    public abstract Integer getId();

    /**
     * Gets the name of the camera.
     * 
     * @return The name of the camera
     */
    public abstract String getName();

    @Override
    public String getOverrideName() {
        return getName();
    }

    public record VisionPoseEstimate(
            int cameraId,
            Pose3d pose,
            double timestamp,
            List<Integer> apriltags,
            double trust,
            double maxDistance
        ) implements StructSerializable {

        public double distanceFrom(VisionPoseEstimate other) {
            return pose.getTranslation().getDistance(other.pose.getTranslation());
        }

        public static VisionPoseEstimate empty(int id) {
            return new VisionPoseEstimate(
                    id,
                    new Pose3d(),
                    0,
                    List.of(),
                    0.0,
                    0.0);
        }

        public Pair<VisionPoseEstimate, VisionEstimateFault> withFault(
                VisionPoseEstimate last,
                Timer jitterTimer,
                Consumer<VisionPoseEstimate> jitterReseter) {
            Translation2d simplePose = pose.getTranslation().toTranslation2d();
            boolean oob = simplePose.getX() < 0.0
                    || simplePose.getX() > FieldConstants.FIELD_LENGTH
                    || simplePose.getY() < 0.0
                    || simplePose.getY() > FieldConstants.FIELD_WIDTH
                    || Double.isNaN(simplePose.getX())
                    || Double.isNaN(simplePose.getY());
            VisionEstimateFault fault = new VisionEstimateFault(
                    oob,
                    maxDistance > 6.0,
                    trust > kVision.AMBIGUITY_CUTOFF,
                    this.distanceFrom(last) > jitterTimer.get() * kSwerve.MAX_DRIVE_VELOCITY,
                    this.apriltags.isEmpty(),
                    Math.abs(pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA,
                    pose.getRotation().getY() > kVision.MAX_ANGLE_DELTA,
                    pose.getRotation().getX() > kVision.MAX_ANGLE_DELTA,
                    false);

            if (!fault.extremeJitter) {
                jitterReseter.accept(this);
            }

            return new Pair<>(this, fault);
        }

        public VisionPoseEstimate withError(double newError) {
            return new VisionPoseEstimate(cameraId, pose, timestamp, apriltags, newError, maxDistance);
        }

        public static class VisionPoseEstimateStruct implements Struct<VisionPoseEstimate> {
            @Override
            public Class<VisionPoseEstimate> getTypeClass() {
                return VisionPoseEstimate.class;
            }

            @Override
            public int getSize() {
                return Integer.BYTES + Pose3d.struct.getSize() + Double.BYTES + /* 16 + */(Double.BYTES * 2);
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] { Pose3d.struct };
            }

            @Override
            public String getSchema() {
                return "int32 poseId; Pose3d pose; double timestamp; double ambiguity; double maxDistance;";
            }

            @Override
            public String getTypeString() {
                return "struct:VisionPoseEstimate";
            }

            @Override
            public void pack(ByteBuffer bb, VisionPoseEstimate value) {
                bb.putInt(value.cameraId);
                Pose3d.struct.pack(bb, value.pose);
                bb.putDouble(value.timestamp);
                bb.putDouble(value.trust);
                bb.putDouble(value.maxDistance);
            }

            @Override
            public VisionPoseEstimate unpack(ByteBuffer bb) {
                int cameraId = bb.getInt();
                Pose3d pose = Pose3d.struct.unpack(bb);
                double timestamp = bb.getDouble();
                double ambiguity = bb.getDouble();
                double maxDistance = bb.getDouble();
                return new VisionPoseEstimate(cameraId, pose, timestamp, new ArrayList<Integer>(), ambiguity,
                        maxDistance);
            }
        }

        public static final VisionPoseEstimateStruct struct = new VisionPoseEstimateStruct();
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
            boolean isDisabled) implements StructSerializable {

        public static VisionEstimateFault empty() {
            return new VisionEstimateFault(false, false, false, false, false, false, false, false, false);
        }

        public boolean isFaulty() {
            return outOfBounds || tooAmbiguous || extremeJitter || noTags || isDisabled;
            // || outOfRange || infeasibleZValue || infeasiblePitchValue || infeasibleRollValue ;
        }

        public static class VisionEstimateFaultStruct implements Struct<VisionEstimateFault> {
            @Override
            public Class<VisionEstimateFault> getTypeClass() {
                return VisionEstimateFault.class;
            }

            @Override
            public int getSize() {
                return 9;
            }

            @Override
            public String getSchema() {
                return "bool outOfBounds; "
                        + "bool outOfRange; "
                        + "bool tooAmbiguous; "
                        + "bool extremeJitter; "
                        + "bool noTags; "
                        + "bool infeasibleZValue; "
                        + "bool infeasiblePitchValue; "
                        + "bool infeasibleRollValue; "
                        + "bool isDisabled;";
            }

            @Override
            public String getTypeString() {
                return "struct:VisionEstimateFault";
            }

            @Override
            public void pack(ByteBuffer bb, VisionEstimateFault value) {
                bb.put(value.outOfBounds ? (byte) 1 : (byte) 0);
                bb.put(value.outOfRange ? (byte) 1 : (byte) 0);
                bb.put(value.tooAmbiguous ? (byte) 1 : (byte) 0);
                bb.put(value.extremeJitter ? (byte) 1 : (byte) 0);
                bb.put(value.noTags ? (byte) 1 : (byte) 0);
                bb.put(value.infeasibleZValue ? (byte) 1 : (byte) 0);
                bb.put(value.infeasiblePitchValue ? (byte) 1 : (byte) 0);
                bb.put(value.infeasibleRollValue ? (byte) 1 : (byte) 0);
                bb.put(value.isDisabled ? (byte) 1 : (byte) 0);
            }

            @Override
            public VisionEstimateFault unpack(ByteBuffer bb) {
                return new VisionEstimateFault(
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1,
                        bb.get() == 1);
            }
        }

        public static final VisionEstimateFaultStruct struct = new VisionEstimateFaultStruct();
    }
}
