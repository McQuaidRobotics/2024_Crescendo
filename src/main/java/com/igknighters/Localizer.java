package com.igknighters;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.plumbing.Channel;
import com.igknighters.util.plumbing.Channel.Receiver;
import com.igknighters.util.plumbing.Channel.Sender;
import com.igknighters.util.plumbing.Channel.ThreadSafetyMarker;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.LogSink;
import monologue.Logged;
import monologue.Monologue;

public class Localizer implements Logged {

    private final Channel<VisionPoseEstimate> visionDataChannel = new Channel<>();
    private final Channel<SwerveDriveSample> swerveDataChannel = new Channel<>();

    private final Receiver<VisionPoseEstimate> visionDataReceiver = visionDataChannel.openReceiver(8, ThreadSafetyMarker.CONCURRENT);
    private final Receiver<SwerveDriveSample> swerveDataReveiver = swerveDataChannel.openReceiver(32, ThreadSafetyMarker.CONCURRENT);

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0, 0, Math.PI * 10);

    private Pose2d latestPose = GeomUtil.POSE2D_CENTER;
    private Pose2d latestVisionPose = GeomUtil.POSE2D_CENTER;
    private double latestVisionTimestamp = 0;

    private final Field2d field;
    public static record NamedPositions(String name, Pose2d[] positions) {}
    private final Channel<NamedPositions> namedPositionsChannel = new Channel<>();
    private final Receiver<NamedPositions> namedPositionsReceiver = namedPositionsChannel.openReceiver(24, ThreadSafetyMarker.SEQUENTIAL);

    private final Channel<ChassisSpeeds> velocityChannel = new Channel<>();

    public Localizer() {
        final var defaultModulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };

        poseEstimator = new SwerveDrivePoseEstimator(
            kSwerve.SWERVE_KINEMATICS,
            GeomUtil.ROTATION2D_ZERO,
            defaultModulePositions,
            GeomUtil.POSE2D_CENTER,
            VecBuilder.fill(0.1, 0.1, 0.1),
            visionStdDevs
        );

        field = new Field2d();
    }

    public void publishField() {
        Monologue.publishSendable("/Visualizers/Field", field, LogSink.NT);
    }

    public Sender<VisionPoseEstimate> visionDataSender() {
        return visionDataChannel.sender();
    }

    public Sender<SwerveDriveSample> swerveDataSender() {
        return swerveDataChannel.sender();
    }

    public Sender<NamedPositions> namedPositionsSender() {
        return namedPositionsChannel.sender();
    }

    public Channel<ChassisSpeeds> velocityChannel() {
        return velocityChannel;
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        Rotation2d gyroAngle = pose.getRotation();
        poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    }

    public void update() {
        while (swerveDataReveiver.hasData()) {
            var sample = swerveDataReveiver.recv();
            poseEstimator.updateWithTime(sample.timestamp(), sample.gyroYaw(), sample.modulePositions());
        }
        while (visionDataReceiver.hasData()) {
            var sample = visionDataReceiver.recv();
            visionStdDevs.set(0, 0, sample.trust());
            visionStdDevs.set(1, 0, sample.trust());
            latestVisionPose = sample.pose().toPose2d();
            log("pose", latestVisionPose);
            latestVisionTimestamp = sample.timestamp();
            poseEstimator.addVisionMeasurement(
                    latestVisionPose,
                    latestVisionTimestamp,
                    visionStdDevs);
        }
        while (namedPositionsReceiver.hasData()) {
            var namedPositions = namedPositionsReceiver.recv();
            field.getObject(namedPositions.name()).setPoses(namedPositions.positions());
        }

        latestPose = poseEstimator.getEstimatedPosition();
        field.getRobotObject().setPose(latestPose);
    }

    public Pose2d pose() {
        return latestPose;
    }

    public Translation2d translation() {
        return latestPose.getTranslation();
    }

    public Pose2d visionPose(double ageLimit) {
        if (latestVisionTimestamp + ageLimit < Timer.getFPGATimestamp()) {
            return latestPose;
        } else {
            return latestVisionPose;
        }
    }
}
