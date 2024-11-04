package com.igknighters;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.TwistyPoseEst;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.plumbing.Channel;
import com.igknighters.util.plumbing.Channel.Receiver;
import com.igknighters.util.plumbing.Channel.Sender;
import com.igknighters.util.plumbing.Channel.ThreadSafetyMarker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final TwistyPoseEst poseEstimator;

    private Pose2d latestPose = GeomUtil.POSE2D_CENTER;
    private Pose2d latestVisionPose = GeomUtil.POSE2D_CENTER;
    private double latestVisionTimestamp = 0;

    private final Field2d field;
    public static record NamedPositions(String name, Pose2d[] positions) {}
    private final Channel<NamedPositions> namedPositionsChannel = new Channel<>();
    private final Receiver<NamedPositions> namedPositionsReceiver = namedPositionsChannel.openReceiver(24, ThreadSafetyMarker.SEQUENTIAL);

    private final Channel<ChassisSpeeds> velocityChannel = new Channel<>();

    public Localizer() {

        poseEstimator = new TwistyPoseEst();

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

    public void reset(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void update() {
        while (swerveDataReveiver.hasData()) {
            var sample = swerveDataReveiver.recv();
            poseEstimator.addDriveSample(
                kSwerve.KINEMATICS,
                sample.modulePositions(),
                sample.timestamp(),
                1.0);
        }
        while (visionDataReceiver.hasData()) {
            var sample = visionDataReceiver.recv();
            latestVisionPose = sample.pose().toPose2d();
            latestVisionTimestamp = sample.timestamp();
            poseEstimator.addVisionSample(
                latestVisionPose,
                latestVisionTimestamp,
                sample.trust()
            );
        }
        while (namedPositionsReceiver.hasData()) {
            var namedPositions = namedPositionsReceiver.recv();
            field.getObject(namedPositions.name()).setPoses(namedPositions.positions());
        }

        latestPose = poseEstimator.getEstimatedPose();
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
