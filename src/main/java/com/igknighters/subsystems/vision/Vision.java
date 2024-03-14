package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.Tracer;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Vision extends SubsystemBase implements Logged {

    private final Camera[] cameras;

    private final BooleanEntry cameraPositionFieldVisualizer;

    private Optional<VisionPoseEstimate> latestEval = Optional.empty();
    private Timer lastEvalTime = new Timer();

    public Vision() {
        this.cameras = List.of(kVision.CAMERA_CONFIGS)
                .stream()
                .map(Camera::create)
                .toArray(Camera[]::new);

        GlobalState.setLocalizer(
                new VisionOnlyPoseEstimator(),
                LocalizerType.Vision);

        cameraPositionFieldVisualizer = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getBooleanTopic("CamerasOnField")
                .getEntry(false);

        cameraPositionFieldVisualizer.accept(false);
    }

    public Optional<VisionPoseEstimate> getLatestEval() {
        return latestEval;
    }

    public Pose2d getLatestPoseWithFallback() {
        return latestEval.map(VisionPoseEstimate::pose)
                .map(Pose3d::toPose2d)
                .orElse(GlobalState.getLocalizedPose());
    }

    @Override
    public void periodic() {

        Tracer.startTrace("VisionPeriodic");
        HashSet<Integer> seenTags = new HashSet<>();

        if (lastEvalTime.hasElapsed(0.2)) {
            latestEval = Optional.empty();
        }

        for (Camera camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();

            if (cameraPositionFieldVisualizer.get(false)) {
                log(
                        camera.getName() + "/3d",
                        GlobalState.getLocalizedPose3d().plus(
                                camera.getRobotToCameraTransform3d()));
                GlobalState.modifyField2d(field -> {
                    Transform2d tf = new Transform2d(
                            camera.getRobotToCameraTransform3d().getTranslation().toTranslation2d(),
                            camera.getRobotToCameraTransform3d().getRotation().toRotation2d());
                    field.getObject(camera.getName()).setPose(
                            field.getRobotPose().plus(tf));
                });
            }

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

            double ambiguity = eval.ambiguity();

            log("cameras/" + camera.getName() + "/rawAmbiguity", ambiguity);

            if (camera.getFaults().outOfRange()) {
                ambiguity *= 2.0;
            }

            if (eval.apriltags().size() < 2) {
                ambiguity *= 2.0;
            }

            ChassisSpeeds velo = GlobalState.getVelocity();
            if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm() > kSwerve.MAX_DRIVE_VELOCITY
                    / 2.0) {
                ambiguity *= 2.0;
            }
            if (velo.omegaRadiansPerSecond > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
                ambiguity *= 2.0;
            }

            Rotation2d rotation = GlobalState.getRobotRot().toRotation2d();
            if (Math.abs(
                MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(eval.pose().getRotation().toRotation2d().getRadians())
                ) > Math.toRadians(30.0)) {
                ambiguity *= 2.0;
            }

            log("cameras/" + camera.getName() + "/ambiguity", ambiguity);

            if (!DriverStation.isAutonomousEnabled())
                GlobalState.submitVisionData(eval, ambiguity);

            seenTags.addAll(eval.apriltags());

            Tracer.endTrace();
        }

        log(
                "SeenTags",
                seenTags.stream().toList().toString());

        GlobalState.modifyField2d(field -> {
            field.getObject("seen_apriltags").setPoses(
                    seenTags.stream()
                            .map(tagId -> FieldConstants.APRIL_TAG_FIELD
                                    .getTagPose(tagId)
                                    .orElseGet(() -> new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d()))
                                    .toPose2d())
                            .toList());
        });

        Tracer.endTrace();
    }
}
