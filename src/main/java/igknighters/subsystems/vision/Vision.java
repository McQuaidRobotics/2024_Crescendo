package igknighters.subsystems.vision;

import igknighters.Localizer;
import igknighters.SimCtx;
import igknighters.constants.RobotConfig;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kVision;
import igknighters.subsystems.SubsystemResources.LockFreeSubsystem;
import igknighters.subsystems.vision.camera.Camera;
import igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import igknighters.util.logging.Tracer;
import igknighters.util.plumbing.Channel.Sender;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import monologue.Monologue;

public class Vision implements LockFreeSubsystem {
    private final Localizer localizer;

    private final Sender<VisionPoseEstimate> visionSender;

    private final Camera[] cameras;

    private final BooleanEntry cameraPositionFieldVisualizer;

    private HashSet<Integer> seenTags = new HashSet<>();

    public Vision(final Localizer localizer, final SimCtx simRobot) {
        this.localizer = localizer;
        this.cameras = List.of(kVision.CameraConfigs.forRobot(RobotConfig.getRobotID()))
                .stream()
                .map(Camera::create)
                .toArray(Camera[]::new);

        for (final var camera : cameras) {
            Monologue.logTree(camera, "/Robot/Vision/" + camera.getName());
        }

        cameraPositionFieldVisualizer = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getBooleanTopic("CamerasOnField")
                .getEntry(false);
        cameraPositionFieldVisualizer.accept(false);

        visionSender = localizer.visionDataSender();
    }

    @Override
    public void periodic() {
        Tracer.startTrace("VisionPeriodic");

        for (Camera camera : cameras) {
            Tracer.startTrace(camera.getName() + "Periodic");
            camera.periodic();

            Pose2d localPose = localizer.pose();

            Optional<VisionPoseEstimate> optEval = camera.evalPose();

            if (!optEval.isPresent() || camera.getFaults().isFaulty()) {
                Tracer.endTrace();
                continue;
            }

            VisionPoseEstimate eval = optEval.get();

            double error = 0.05;

            error += Math.pow(Math.sqrt(eval.trust()), 3.0);

            log("cameras/" + camera.getName() + "/rawAmbiguity", error);

            if (camera.getFaults().outOfRange()) {
                error *= 2.0;
            }

            if (eval.apriltags().size() < 2) {
                error *= 2.0;
            }

            ChassisSpeeds velo = localizer.speeds();
            if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm() > kSwerve.MAX_DRIVE_VELOCITY
                    / 2.0) {
                error *= 2.0;
            }
            if (velo.omegaRadiansPerSecond > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
                error *= 2.0;
            }

            Rotation2d rotation = localPose.getRotation();
            if (Math.abs(
                MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(eval.pose().getRotation().toRotation2d().getRadians())
                ) > Math.toRadians(30.0)) {
                error *= 2.0;
            }

            log("cameras/" + camera.getName() + "/error", error);

            visionSender.send(eval.withError(error));

            seenTags.addAll(eval.apriltags());

            Tracer.endTrace();
        }

        log("seenTags", seenTags.stream().mapToInt(i -> i).toArray());

        seenTags.clear();

        Tracer.endTrace();
    }
}
