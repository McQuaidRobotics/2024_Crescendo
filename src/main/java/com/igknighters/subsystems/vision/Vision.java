package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.Tracer;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final List<Camera> cameras;

    private final BooleanEntry visionFieldVisualizer;

    public Vision() {
        this.cameras = List.of(kVision.CAMERA_CONFIGS)
                .stream()
                .map(Camera::create)
                .toList();

        GlobalState.setLocalizer(
                new VisionOnlyPoseEstimator(),
                LocalizerType.Vision);

        visionFieldVisualizer = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getBooleanTopic("CamerasOnField")
                .getEntry(false);

        visionFieldVisualizer.accept(false);
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");
        HashSet<Integer> seenTags = new HashSet<>();

        for (Camera camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();

            if (visionFieldVisualizer.get(false)) {
                GlobalState.modifyField2d(field -> {
                    Transform2d tf = new Transform2d(
                            camera.getRobotToCameraTransform3d().getTranslation().toTranslation2d(),
                            camera.getRobotToCameraTransform3d().getRotation().toRotation2d());
                    field.getObject(camera.getName()).setPose(
                            field.getRobotPose().plus(tf));
                });
            }

            Optional<VisionPoseEstimate> optEval = camera.evalPose();

            if (!optEval.isPresent()) {
                Tracer.endTrace();
                continue;
            }

            VisionPoseEstimate eval = optEval.get();

            double ambiguity = eval.ambiguity;

            if (eval.apriltags.isEmpty() || ambiguity > 0.5 || eval.maxDistance > 5.5) {
                Tracer.endTrace();
                continue;
            }

            if (eval.apriltags.size() < 2) {
                ambiguity *= 2.0;
            }

            if (Math.abs(eval.pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA) {
                // The cameras height does not change typically, so if it does, it is likely a
                // false positive
                Logger.recordOutput("/Vision/" + camera.getName() + "/WeirdZ", true);
                ambiguity *= 2.0;
            }

            GlobalState.submitVisionData(eval, ambiguity);

            seenTags.addAll(eval.apriltags);

            Tracer.endTrace();
        }

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
