package com.igknighters.subsystems.vision;

import com.igknighters.Localizer;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.SubsystemResources.LockFreeSubsystem;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.logging.Tracer;
import com.igknighters.util.plumbing.Channel.Sender;
import com.igknighters.util.plumbing.Channel.Receiver;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import monologue.Monologue;

public class Vision implements LockFreeSubsystem {
    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();

    private final Localizer localizer;

    private final Receiver<ChassisSpeeds> velocityReceiver;
    private final Sender<VisionPoseEstimate> visionSender;

    private final Camera[] cameras;

    private final BooleanEntry cameraPositionFieldVisualizer;

    private HashSet<Integer> seenTags = new HashSet<>();

    public Vision(final Localizer localizer) {
        this.localizer = localizer;
        this.cameras = List.of(kVision.CAMERA_CONFIGS)
                .stream()
                .map(Camera::create)
                .toArray(Camera[]::new);

        for (final var camera : cameras) {
            Monologue.logTree(camera, "/Robot/Vision/" + camera.getName());
        }

        cameraPositionFieldVisualizer = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getBooleanTopic("CamerasOnField")
                .getEntry(false);
        cameraPositionFieldVisualizer.accept(false);

        velocityReceiver = localizer.velocityChannel().openReceiver(1);
        visionSender = localizer.visionDataSender();
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");

        for (Camera camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();

            Pose2d localPose = localizer.pose();

            Optional<VisionPoseEstimate> optEval = camera.evalPose();

            if (!optEval.isPresent() || camera.getFaults().isFaulty()) {
                Tracer.endTrace();
                continue;
            }

            VisionPoseEstimate eval = optEval.get();

            double error = 0.05;

            error += Math.pow(Math.sqrt(eval.trust()), 3.0);

            log("cameras/" + camera.getName() + "/rawAmbiguity", error);

            if (camera.getFaults().outOfRange()) {
                error *= 2.0;
            }

            if (eval.apriltags().size() < 2) {
                error *= 2.0;
            }

            ChassisSpeeds velo = velocityReceiver.inspectOrDefault(ZERO_SPEEDS);
            if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm() > kSwerve.MAX_DRIVE_VELOCITY
                    / 2.0) {
                error *= 2.0;
            }
            if (velo.omegaRadiansPerSecond > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
                error *= 2.0;
            }

            Rotation2d rotation = localPose.getRotation();
            if (Math.abs(
                MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(eval.pose().getRotation().toRotation2d().getRadians())
                ) > Math.toRadians(30.0)) {
                error *= 2.0;
            }

            log("cameras/" + camera.getName() + "/error", error);

            visionSender.send(eval.withError(error));

            seenTags.addAll(eval.apriltags());

            Tracer.endTrace();
        }

        log("seenTags", seenTags.stream().mapToInt(i -> i).toArray());

        seenTags.clear();

        Tracer.endTrace();
    }
}
