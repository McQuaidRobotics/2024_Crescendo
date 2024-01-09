package com.igknighters.vision;

import java.util.List;

import org.photonvision.estimation.TargetModel;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

public class VisionVerifier {

    private VisionPoseEstimator visionPoseEstimator;
    private Notifier visionNotifier;

    public void startVision() {
        visionPoseEstimator = new VisionPoseEstimator(
            new AprilTagFieldLayout(List.of(), 100.0, 100.0),
            TargetModel.kAprilTag36h11,
            kVision.CAMERAS
        );
        visionNotifier.setCallback(this::processVision);
        visionNotifier.startPeriodic(0.01);
    }

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
                case 2:
                    return TRUSTED;
                case 1:
                    return SemiTrusted;
                case 0:
                    return UnTrusted;
                default:
                    return UnTrusted;
            }
        }
    }


    private void processVision() {
        var estimatedPoses = visionPoseEstimator.estimateCurrentPosition();
        if (estimatedPoses.size() == 0) {
            return;
        }

        var poseTrustVals = new int[estimatedPoses.size()];
        for (int i = 0; i < poseTrustVals.length; i++) {
            poseTrustVals[i] = 0;
        }

        // for now ignore history
        // if all values received are within `kVision.ToleratedMultiCamDifference` of each other,
        // then we can trust the values at level `SemiTrusted`

        for (int i = 0; i < estimatedPoses.size(); i++) {
            var pose = estimatedPoses.get(i).estimatedPose;
            for (int j = i + 1; j < estimatedPoses.size(); j++) {
                var otherPose = estimatedPoses.get(j).estimatedPose;
                if (pose.getTranslation().minus(otherPose.getTranslation()).getNorm() < kVision.ToleratedMultiCamDifference) {
                    poseTrustVals[i]++;
                }
            }
        }

        var i = 0;
        for (var estPose : estimatedPoses) {
            var trust = VisionTrustworthiness.formInt(poseTrustVals[i]);
            GlobalState.submitVisionData(estPose, trust.stdDevs);
            i++;
        }
    }
}
