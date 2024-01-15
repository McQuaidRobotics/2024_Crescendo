package com.igknighters.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.igknighters.vision.camera.Camera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionPoseEstimator {

    private AprilTagFieldLayout aprilTagFieldLayout;
    private TargetModel tagModel;
    private Camera[] cameras;

    public VisionPoseEstimator(AprilTagFieldLayout fieldLayout, TargetModel model, Camera... cameras) {
        aprilTagFieldLayout = fieldLayout;
        tagModel = model;
        if (cameras.length < 1) {
            DriverStation.reportWarning("[VisionPoseEstimator] no cameras provided", false);
        }
        if (RobotBase.isReal()) {
            this.cameras = cameras;
        }
    }

    public List<EstimatedRobotPose> estimateCurrentPosition() {
        if (RobotBase.isSimulation()) {
            return List.of();
        }
        if (cameras.length == 0) {
            return List.of();
        }
        var results = new ArrayList<Camera.CameraResult>();
        for (var camera : cameras) {
            results.add(camera.getLatestResult());
        }
        return estimateCurrentPositionMultiCamera(results);
    }

    private List<EstimatedRobotPose> estimateCurrentPositionMultiCamera(List<Camera.CameraResult> results) {
        var estimatedPoses = new ArrayList<EstimatedRobotPose>();
        for (var result : results) {
            if (!result.isValid()) {
                continue;
            }
            var optionalPose = multiTagSolve(result);
            if (optionalPose.isPresent()) {
                estimatedPoses.add(optionalPose.get());
            }
        }
        return estimatedPoses;
    }

    private Optional<EstimatedRobotPose> multiTagSolve(Camera.CameraResult result) {
        if (!result.isValid() || !result.getCamera().hasCalibrationData()) {
            return Optional.empty();
        }
        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();

        for (var target : result.getTargets()) {
            var optionalTagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (optionalTagPose.isEmpty()) {
                DriverStation.reportWarning("[VisionPoseEstimator] saw unknow apriltag id: " + target.getFiducialId(),
                    false);
                continue;
            }

            var tagPose = optionalTagPose.get();
            visCorners.addAll(target.getDetectedCorners());
            knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
        }

        if (knownVisTags.size() < 2) {
            return singleTagSolve(result);
        }

        //so ugly but only is called once
        var cameraMatrix = result.getCamera().getCameraMatrix().get();
        var distCoeffs = result.getCamera().getDistCoeffs().get();

        var pnpResult = VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, result.getTargets(), aprilTagFieldLayout, tagModel);

        var best = new Pose3d()
            .plus(pnpResult.best)
            .plus(result.getCameraTransform3d().inverse());

        return Optional.of(new EstimatedRobotPose(best, result.getTimestamp(), result.getTargets(), PoseStrategy.MULTI_TAG_PNP_ON_RIO));
    }

    private Optional<EstimatedRobotPose> singleTagSolve(Camera.CameraResult result) {
        if (!result.isValid()) {
            return Optional.empty();
        }

        PhotonTrackedTarget lowestAmbiguityTarget = null;

        double lowestAmbiguityScore = 10;

        for (PhotonTrackedTarget target : result.getTargets()) {
            double targetPoseAmbiguity = target.getPoseAmbiguity();
            // Make sure the target is a Fiducial target.
            if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                lowestAmbiguityScore = targetPoseAmbiguity;
                lowestAmbiguityTarget = target;
            }
        }

        if (lowestAmbiguityTarget == null) return Optional.empty();

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

        Optional<Pose3d> targetPosition = aprilTagFieldLayout.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
                new EstimatedRobotPose(
                        targetPosition
                                .get()
                                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                                .transformBy(result.getCameraTransform3d().inverse()),
                        result.getTimestamp(),
                        result.getTargets(), PoseStrategy.LOWEST_AMBIGUITY));
    }

    public void updateField(AprilTagFieldLayout field) {
        this.aprilTagFieldLayout = field;
    }
}
