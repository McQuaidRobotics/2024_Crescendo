package com.igknighters.subsystems.vision;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.LocalizerType;
import com.igknighters.constants.ConstValues.kVision;
import com.igknighters.subsystems.vision.camera.Camera;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
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
                case 1:
                    return UnTrusted;
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
        for (var camera : cameras) {
            var optEval = camera.evalPose();
            if (optEval.isPresent()) {
                var eval = optEval.get();

                var trust = VisionTrustworthiness.formInt(Math.min(eval.apriltags.length, 3));

                GlobalState.submitVisionData(eval, trust.stdDevs);

                //TODO: Add showing april tags on field
            }
        }
    }
}
