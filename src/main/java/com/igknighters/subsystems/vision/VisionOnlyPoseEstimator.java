package com.igknighters.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A fake pose estimator that only uses vision.
 */
public class VisionOnlyPoseEstimator extends PoseEstimator<VisionOnlyPoseEstimator.FakeWheelPositions> {
    public static class FakeWheelPositions implements WheelPositions<FakeWheelPositions> {
        @Override
        public FakeWheelPositions copy() {
            return new FakeWheelPositions();
        }

        @Override
        public FakeWheelPositions interpolate(FakeWheelPositions endValue, double t) {
            return new FakeWheelPositions();
        }
    }

    public static class FakeKinematics implements Kinematics<Double, FakeWheelPositions> {
        @Override
        public ChassisSpeeds toChassisSpeeds(Double wheelSpeeds) {
            return new ChassisSpeeds();
        }

        @Override
        public Double toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
            return 0.0;
        }

        @Override
        public Twist2d toTwist2d(FakeWheelPositions start, FakeWheelPositions end) {
            return new Twist2d();
        }
    }

    public static class FakeOdometry extends Odometry<FakeWheelPositions> {
        public FakeOdometry() {
            super(new FakeKinematics(), new Rotation2d(), new FakeWheelPositions(), new Pose2d());
        }
    }

    Pose2d lastPose = new Pose2d();
    Pose2d lastPose2 = new Pose2d();

    public VisionOnlyPoseEstimator() {
        super(
                new FakeKinematics(),
                new FakeOdometry(),
                VecBuilder.fill(0.01, 0.01, 0.01),
                VecBuilder.fill(1.0, 1.0, 1.0));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        lastPose2 = lastPose;
        lastPose = visionRobotPoseMeters;
    }

    @Override
    public Pose2d getEstimatedPosition() {
        var trans = lastPose
                .getTranslation()
                .plus(lastPose2.getTranslation())
                .div(2.0);
        var rot = lastPose.getRotation();

        return new Pose2d(trans, rot);
    }
}
