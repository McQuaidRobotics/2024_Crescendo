package com.igknighters.subsystems.vision.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.igknighters.constants.FieldConstants;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

/**
 * An abstraction for a photon camera.
 */
public class CameraRealPhoton extends Camera {
    private final PhotonCamera camera;
    private final Integer id;
    private final Transform3d cameraPose;
    private final PhotonPoseEstimator poseEstimator;

    private final VisionPoseEstimate noPoseEst;

    private VisionPoseEstimate previousPoseEst;
    private Timer previousPoseTimer;

    /**
     * Creates an abstraction for a photon camera.
     * 
     * @param cameraName The name of the camera
     * @param id         The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     */
    public CameraRealPhoton(String cameraName, Integer id, Transform3d cameraPose) {
        super(id);
        this.camera = new PhotonCamera(cameraName);
        this.id = id;
        this.cameraPose = cameraPose;

        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.APRIL_TAG_FIELD,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                this.camera,
                this.cameraPose);
        poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
        noPoseEst = VisionPoseEstimate.empty(id);

        BootupLogger.bootupLog("    " + cameraName + " camera initialized (real)");
    }

    private Optional<VisionPoseEstimate> realEvaluatePose() {
        Optional<EstimatedRobotPose> opt = poseEstimator.update();

        if (!opt.isPresent()) {
            log("poseEst", noPoseEst);
            return Optional.empty();
        }

        EstimatedRobotPose estRoboPose = opt.get();

        log("photonPoseEst", estRoboPose.estimatedPose);

        List<Integer> targetIds = estRoboPose.targetsUsed
                .stream()
                .map(PhotonTrackedTarget::getFiducialId)
                .toList();

        double avgAmbiguity = estRoboPose.targetsUsed
                .stream()
                .map(PhotonTrackedTarget::getPoseAmbiguity)
                .reduce(0.0, Double::sum)
                / estRoboPose.targetsUsed.size();

        double maxDistance = estRoboPose.targetsUsed
                .stream()
                .map(PhotonTrackedTarget::getBestCameraToTarget)
                .map(Transform3d::getTranslation)
                .map(Translation3d::toTranslation2d)
                .map(Translation2d::getNorm)
                .reduce(0.0, Math::max);

        return Optional.of(
            log(
                "poseEst",
                new VisionPoseEstimate(
                    this.id,
                    estRoboPose.estimatedPose,
                    estRoboPose.timestampSeconds,
                    targetIds,
                    avgAmbiguity,
                    maxDistance)
        ));
    }

    @Override
    public Transform3d getRobotToCameraTransform3d() {
        return cameraPose;
    }

    @Override
    public Integer getId() {
        return id;
    }

    @Override
    public String getName() {
        return camera.getName();
    }

    private void resetLastPoseInfo(VisionPoseEstimate poseEst) {
        previousPoseEst = poseEst;
        previousPoseTimer.restart();
    }

    @Override
    public void periodic() {

        if (previousPoseEst == null) {
            var eval = realEvaluatePose();
            if (eval.isPresent()) {
                previousPoseEst = eval.get();
                previousPoseTimer = new Timer();
                previousPoseTimer.start();
            }
            this.update(
                eval.map(est -> Pair.of(est, VisionEstimateFault.empty())),
                camera.isConnected()
            );
        } else {
            this.update(realEvaluatePose()
                .map(est -> est.withFault(previousPoseEst, previousPoseTimer, this::resetLastPoseInfo)),
            camera.isConnected()
            );
        }
    }
}
