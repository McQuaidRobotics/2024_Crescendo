package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.AprilTags;
import com.igknighters.constants.ConstValues.kDimensions;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.util.Tracer;

import java.util.List;

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
            LocalizerType.VISION
        );
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");
        for (var camera : cameras) {
            camera.periodic();
            var optEval = camera.evalPose();
            if (optEval.isPresent()) {
                var eval = optEval.get();

                if (Math.abs(eval.pose.getTranslation().getZ() - kDimensions.BELLYPAN_HEIGHT) > kVision.MAX_Z_DELTA) {
                    continue;
                }

                GlobalState.submitVisionData(eval, eval.ambiguity);

                GlobalState.modifyField(field -> {
                    field.getObject("seen_apriltags").setPoses(
                        eval.apriltags.stream()
                            .map(tagId -> AprilTags.APRIL_TAG_FIELD
                                .getTagPose(tagId)
                                .orElseGet(() -> new Pose3d(new Translation3d(-10.0, -10.0, -10.0), new Rotation3d()))
                                .toPose2d()
                            )
                            .toList()
                    );
                });
            }
        }
        Tracer.endTrace();
    }
}
