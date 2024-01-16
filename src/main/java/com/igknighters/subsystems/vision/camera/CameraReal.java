package com.igknighters.subsystems.vision.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.igknighters.util.BootupLogger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * An abstraction for a photon camera.
 */
public class CameraReal implements Camera {
    private final PhotonCamera camera;
    private final Integer id;
    private final Transform3d cameraPose;
    private final PhotonPoseEstimator poseEstimator;

    private static final AprilTagFieldLayout TEMP_APRIL_TAGS = new AprilTagFieldLayout(List.of(
            new AprilTag(8, new Pose3d(
                    new Translation3d(0, 1.4511, 5.772),
                    new Rotation3d(0, 0, 0)))),
            100.0,
            100.0);

    /**
     * Creates an abstraction for a photon camera.
     * 
     * @param cameraName The name of the camera
     * @param id         The ID of the camera
     * @param cameraPose The pose of the camera relative to the robot
     */
    public CameraReal(String cameraName, Integer id, Pose3d cameraPose) {
        this.camera = new PhotonCamera(cameraName);
        this.id = id;
        this.cameraPose = new Transform3d(cameraPose.getTranslation(), cameraPose.getRotation());

        poseEstimator = new PhotonPoseEstimator(
                TEMP_APRIL_TAGS,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                this.camera,
                this.cameraPose);
        poseEstimator.setTagModel(TargetModel.kAprilTag36h11);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        BootupLogger.BootupLog(cameraName + " camera initialized");
    }

    @Override
    public Optional<VisionPoseEst> evalPose() {
        return poseEstimator.update()
                .map(estRoboPose -> new VisionPoseEst(
                        this.id,
                        estRoboPose.estimatedPose,
                        estRoboPose.timestampSeconds,
                        estRoboPose.targetsUsed
                            .stream()
                            .mapToInt(PhotonTrackedTarget::getFiducialId)
                            .toArray()
                    )
                );
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
}
