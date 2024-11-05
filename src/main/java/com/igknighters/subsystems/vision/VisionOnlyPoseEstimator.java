package com.igknighters.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A fake pose estimator that only uses vision.
 */
public class VisionOnlyPoseEstimator extends PoseEstimator<Double[]> {

    public static class FakeKinematics implements Kinematics<Double, Double[]> {
        @Override
        public ChassisSpeeds toChassisSpeeds(Double wheelSpeeds) {
            return new ChassisSpeeds();
        }

        @Override
        public Double toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
            return 0.0;
        }

        @Override
        public Twist2d toTwist2d(Double[] start, Double[] end) {
            return new Twist2d();
        }

        @Override
        public Double[] copy(Double[] positions) {
            return new Double[0];
        }

        @Override
        public void copyInto(Double[] positions, Double[] output) {
        }

        @Override
        public Double[] interpolate(Double[] startValue, Double[] endValue, double t) {
            return new Double[0];
        }
    }

    public static class FakeOdometry extends Odometry<Double[]> {
        public FakeOdometry() {
            super(new FakeKinematics(), new Rotation2d(), new Double[0], new Pose2d());
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
