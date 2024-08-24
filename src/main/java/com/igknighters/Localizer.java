package com.igknighters;

import com.igknighters.constants.ConstValues.kChannels;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.odometryThread.SwerveDriveSample;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.plumbing.Channels.Receiver;
import com.igknighters.util.plumbing.Channels.Sender;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


public class Localizer {
    private final Sender<Pose2d> positionSender = Sender.broadcast(kChannels.POSITION, Pose2d.class);
    private final Receiver<VisionPoseEstimate> visionDataReceiver = Receiver.buffered(kChannels.VISION, 32, VisionPoseEstimate.class);
    private final Receiver<SwerveDriveSample> swerveDataReveiver = Receiver.buffered(kChannels.SWERVE_ODO_SAMPLES, 32, SwerveDriveSample.class);

    private final SwerveDrivePoseEstimator hybridPoseEstimator;

    private final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0, 0, 1.0);

    private Pose2d latestPose = GeomUtil.POSE2D_CENTER;

    public Localizer() {
        final var defaultModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        hybridPoseEstimator = new SwerveDrivePoseEstimator(
            kSwerve.SWERVE_KINEMATICS,
            GeomUtil.ROTATION2D_ZERO,
            defaultModulePositions,
            GeomUtil.POSE2D_CENTER,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(1.0, 1.0, 1.0)
        );
    }

    public void resetOdometry(Pose2d pose, SwerveModulePosition[] modulePositions) {
        Rotation2d gyroAngle = pose.getRotation();
        hybridPoseEstimator.resetPosition(gyroAngle, modulePositions, pose);
    }

    public void update() {
        while (swerveDataReveiver.hasData()) {
            var sample = swerveDataReveiver.recv();
            hybridPoseEstimator.updateWithTime(sample.timestamp(), sample.gyroYaw(), sample.modulePositions());
        }
        while (visionDataReceiver.hasData()) {
            var sample = visionDataReceiver.recv();
            visionStdDevs.set(0, 0, sample.trust());
            visionStdDevs.set(1, 0, sample.trust());
            hybridPoseEstimator.addVisionMeasurement(
                sample.pose().toPose2d(),
                sample.timestamp(),
                visionStdDevs
            );
        }

        latestPose = hybridPoseEstimator.getEstimatedPosition();
        positionSender.send(latestPose);
        // visionPositionSender.send(visionOnlyEstimator.getEstimatedPosition());
        // swervePositionSender.send(swerveOnlyEstimator.getPoseMeters());
    }

    public Pose2d pose() {
        return latestPose;
    }
}
