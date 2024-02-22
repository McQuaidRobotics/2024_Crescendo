package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.util.Tracer;

import java.util.HashSet;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final List<Camera> cameras;

    public Vision() {
        this.cameras = List.of(kVision.CAMERA_CONFIGS)
                .stream()
                .map(Camera::create)
                .toList();

        GlobalState.setLocalizer(
                new VisionOnlyPoseEstimator(),
                LocalizerType.Vision);
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");
        HashSet<Integer> seenTags = new HashSet<>();
        for (var camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();
            var optEval = camera.evalPose();
            if (optEval.isPresent()) {
                var eval = optEval.get();

                if (Math.abs(eval.pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA) {
                    // The cameras height does not change typically, so if it does, it is likely a
                    // false positive
                    Logger.recordOutput("/Vision/" + camera.getName() + "/WeirdZ", true);
                    GlobalState.submitVisionData(eval, Math.min(eval.ambiguity * 3.0, 1.0));
                } else {
                    GlobalState.submitVisionData(eval, eval.ambiguity);
                }

                seenTags.addAll(eval.apriltags);
            }
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
