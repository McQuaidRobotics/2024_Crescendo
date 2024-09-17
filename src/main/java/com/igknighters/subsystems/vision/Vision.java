package com.igknighters.subsystems.vision;

import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.SubsystemResources.LockFreeSubsystem;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.logging.Tracer;
import com.igknighters.util.plumbing.Channels.Receiver;
import com.igknighters.util.plumbing.Channels.Sender;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Vision implements LockFreeSubsystem {
    private final Receiver<Pose2d> poseReceiver = Receiver.latest(kChannels.POSITION, Pose2d.class);
    private final Receiver<ChassisSpeeds> velocityReceiver = Receiver.latest(kChannels.VELOCITY, ChassisSpeeds.class);
    private final Sender<VisionPoseEstimate> visionSender = Sender.broadcast(kChannels.VISION, VisionPoseEstimate.class);
    private final Sender<int[]> apriltagSender = Sender.broadcast(kChannels.APRILTAGS, int[].class);

    private final Camera[] cameras;

    private final BooleanEntry cameraPositionFieldVisualizer;

    private Optional<VisionPoseEstimate> latestEval = Optional.empty();
    private Timer lastEvalTime = new Timer();
    private HashSet<Integer> seenTags = new HashSet<>();

    public Vision() {
        this.cameras = List.of(kVision.CAMERA_CONFIGS)
                .stream()
                .map(Camera::create)
                .toArray(Camera[]::new);

        cameraPositionFieldVisualizer = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getBooleanTopic("CamerasOnField")
                .getEntry(false);

        cameraPositionFieldVisualizer.accept(false);
    }

    private Pose2d getLocalizedPose() {
        return poseReceiver.inspectOrDefault(GeomUtil.POSE2D_CENTER);
    }

    public Optional<VisionPoseEstimate> getLatestEval() {
        return latestEval;
    }

    public Pose2d getLatestPoseWithFallback() {
        return latestEval.map(VisionPoseEstimate::pose)
                .map(Pose3d::toPose2d)
                .orElseGet(this::getLocalizedPose);
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");

        if (lastEvalTime.hasElapsed(0.2)) {
            latestEval = Optional.empty();
        }

        for (Camera camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();

            Pose2d localPose = getLocalizedPose();

            Optional<VisionPoseEstimate> optEval = camera.evalPose();

            if (!optEval.isPresent() || camera.getFaults().isFaulty()) {
                Tracer.endTrace();
                continue;
            }

            VisionPoseEstimate eval = optEval.get();

            if (latestEval.isEmpty() || eval.timestamp() > latestEval.get().timestamp()) {
                latestEval = Optional.of(eval);
                lastEvalTime.restart();
            }

            double ambiguity = eval.trust();

            log("cameras/" + camera.getName() + "/rawAmbiguity", ambiguity);

            if (camera.getFaults().outOfRange()) {
                ambiguity *= 2.0;
            }

            if (eval.apriltags().size() < 2) {
                ambiguity *= 2.0;
            }

            if (velocityReceiver.hasData()) {
                ChassisSpeeds velo = velocityReceiver.inspect();
                if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm() > kSwerve.MAX_DRIVE_VELOCITY
                        / 2.0) {
                    ambiguity *= 2.0;
                }
                if (velo.omegaRadiansPerSecond > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
                    ambiguity *= 2.0;
                }
            }

            Rotation2d rotation = localPose.getRotation();
            if (Math.abs(
                MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(eval.pose().getRotation().toRotation2d().getRadians())
                ) > Math.toRadians(30.0)) {
                ambiguity *= 2.0;
            }

            log("cameras/" + camera.getName() + "/ambiguity", ambiguity);

            visionSender.send(eval.withAmbiguity(ambiguity));

            seenTags.addAll(eval.apriltags());

            Tracer.endTrace();
        }

        int[] seenTagsArray = seenTags.stream().mapToInt(i -> i).toArray();
        apriltagSender.send(seenTagsArray);
        log("seenTags", seenTagsArray);

        seenTags.clear();

        Tracer.endTrace();
    }
}
