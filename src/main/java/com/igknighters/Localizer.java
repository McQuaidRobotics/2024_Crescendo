package com.igknighters;

import java.util.List;

import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.TwistyPoseEst;
import com.igknighters.util.logging.Tracer;
import com.igknighters.util.plumbing.Channel;
import com.igknighters.util.plumbing.Channel.Receiver;
import com.igknighters.util.plumbing.Channel.Sender;
import com.igknighters.util.plumbing.Channel.ThreadSafetyMarker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import monologue.LogSink;
import monologue.LogLocal;
import monologue.Monologue;
import monologue.Annotations.Log;

public class Localizer implements LogLocal {

    private final Channel<VisionPoseEstimate> visionDataChannel = new Channel<>(new VisionPoseEstimate[0]);
    private final Channel<SwerveDriveSample> swerveDataChannel = new Channel<>(new SwerveDriveSample[0]);

    private final Receiver<VisionPoseEstimate> visionDataReceiver = visionDataChannel.openReceiver(8, ThreadSafetyMarker.CONCURRENT);
    private final Receiver<SwerveDriveSample> swerveDataReveiver = swerveDataChannel.openReceiver(32, ThreadSafetyMarker.CONCURRENT);

    private final TwistyPoseEst poseEstimator;

    @Log(key = "pose")
    private Pose2d latestPose = FieldConstants.POSE2D_CENTER;
    @Log(key = "speeds")
    private ChassisSpeeds latestSpeeds = new ChassisSpeeds();
    @Log(key = "visionPose")
    private Pose2d latestVisionPose = FieldConstants.POSE2D_CENTER;
    @Log(key = "visionTimestamp")
    private double latestVisionTimestamp = 0;

    private final Field2d field;
    public static record NamedPositions(String name, Pose2d[] positions) {}
    private final Channel<NamedPositions> namedPositionsChannel = new Channel<>(new NamedPositions[0]);
    private final Receiver<NamedPositions> namedPositionsReceiver = namedPositionsChannel.openReceiver(24, ThreadSafetyMarker.SEQUENTIAL);

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

    public void reset(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void update() {
        Tracer.startTrace("SwerveSamples");
        final SwerveDriveSample[] swerveSamples = log("swerveSamples", swerveDataReveiver.recvAll());
        for (final SwerveDriveSample sample : swerveSamples) {
            poseEstimator.addDriveSample(
                kSwerve.KINEMATICS,
                sample.modulePositions(),
                sample.gyroYaw(),
                sample.timestamp(),
                1.0
            );
        }
        Tracer.endTrace();
        Tracer.startTrace("VisionSamples");
        final List<VisionPoseEstimate> unsortedVisionSamples = List.of(log("visionSamples", visionDataReceiver.recvAll()));
        final List<VisionPoseEstimate> visionSamples = unsortedVisionSamples.stream()
            .sorted((a, b) -> Double.compare(a.timestamp(), b.timestamp()))
            .toList();
        for (final VisionPoseEstimate sample : visionSamples) {
            latestVisionPose = sample.pose().toPose2d();
            latestVisionTimestamp = sample.timestamp();
            poseEstimator.addVisionSample(
                latestVisionPose,
                latestVisionTimestamp,
                sample.trust()
            );
        }
        Tracer.endTrace();

        Tracer.startTrace("Prune");
        poseEstimator.prune(0.25);
        Tracer.endTrace();

        while (namedPositionsReceiver.hasData()) {
            var namedPositions = namedPositionsReceiver.recv();
            field.getObject(namedPositions.name()).setPoses(namedPositions.positions());
        }

        latestPose = Tracer.traceFunc("ReadEstPose", poseEstimator::getEstimatedPose);
        field.getRobotObject().setPose(latestPose);

        Pose2d poseFromABitAgo = poseEstimator.getEstimatedPoseFromPast(0.05);
        Twist2d twist = poseFromABitAgo.log(latestPose);
        latestSpeeds = new ChassisSpeeds(
            twist.dx / 0.05,
            twist.dy / 0.05,
            twist.dtheta / 0.05
        );
    }

    public Pose2d pose() {
        return latestPose;
    }

    public ChassisSpeeds speeds() {
        return latestSpeeds;
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
