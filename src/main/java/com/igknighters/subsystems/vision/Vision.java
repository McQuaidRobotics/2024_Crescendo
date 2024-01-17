package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.AprilTags;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;
import com.igknighters.util.Tracer;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final List<Camera> cameras;

    private enum VisionTrustworthiness {
        TRUSTED(kVision.visionStdDevsReal),
        SemiTrusted(kVision.visionStdDevsTrust),
        UnTrusted(kVision.visionStdDevs);

        final Vector<N3> stdDevs;

        private VisionTrustworthiness(Vector<N3> stdDevs) {
            this.stdDevs = stdDevs;
        }

        private static VisionTrustworthiness formInt(int i) {
            switch (i) {
                case 3:
                    return TRUSTED;
                case 2:
                    return SemiTrusted;
                default:
                    return UnTrusted;
            }
        }
    }

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

                var trust = VisionTrustworthiness.formInt(Math.min(eval.apriltags.size(), 3));

                SmartDashboard.putString(camera.getName(), eval.pose.toString());

                GlobalState.submitVisionData(eval, trust.stdDevs);

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
