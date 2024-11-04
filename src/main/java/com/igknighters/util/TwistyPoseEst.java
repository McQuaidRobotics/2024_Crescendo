package com.igknighters.util;

import java.util.LinkedList;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.Timer;

public class TwistyPoseEst {
    private static final double kMaxSampleAge = 0.3;

    private final class TimestampedTwist2d extends Twist2d {
        public final double timestamp;

        public TimestampedTwist2d(double dx, double dy, double dtheta, double timestamp) {
            super(dx, dy, dtheta);
            this.timestamp = timestamp;
        }
    }

    private final LinkedList<TimestampedTwist2d> samples = new LinkedList<>();
    private Pose2d rootPose = new Pose2d();
    private WheelPositions<?> prevWheelPositions;

    public void resetPose(Pose2d pose) {
        rootPose = pose;
        samples.clear();
    }

    private OptionalInt indexForTimestamp(double timestamp) {
        if (samples.getFirst().timestamp > timestamp) {
            return OptionalInt.empty();
        }
        if (samples.getLast().timestamp < timestamp) {
            return OptionalInt.of(samples.size() - 1);
        }
        int low = 0;
        int high = samples.size() - 1;
        while (low <= high) {
            int mid = (low + high) / 2;
            double midTime = samples.get(mid).timestamp;
            if (midTime < timestamp) {
                low = mid + 1;
            } else if (midTime > timestamp) {
                high = mid - 1;
            } else {
                return OptionalInt.of(mid);
            }
        }
        return OptionalInt.of(low - 1);
    }

    private Pose2d poseAtIndex(int index) {
        // switching this over to primitive math would be a good idea
        Pose2d pose = rootPose;
        for (int i = 0; i < index; i++) {
            pose = pose.exp(samples.get(i));
        }
        return pose;
    }

    private void pruneToRoot() {
        while (!samples.isEmpty() && samples.getFirst().timestamp < Timer.getFPGATimestamp() - kMaxSampleAge) {
            rootPose = rootPose.exp(samples.removeFirst());
        }
    }

    /**
     * Adds a sample to the estimator
     * 
     * @param pose the pose of the robot at the time of the sample
     * @param timestamp the timestamp of the sample
     * @param weight the weight of the sample (0.0 to 1.0)
     */
    public void addVisionSample(Pose2d pose, double timestamp, double weight) {
        var opt = indexForTimestamp(timestamp);
        if (opt.isEmpty()) {
            // timestamp is before the first sample
            return;
        }
        int index = opt.getAsInt();

        Pose2d lastPose = poseAtIndex(index);
        Twist2d twist = lastPose.log(pose);
        samples.add(
            index,
            new TimestampedTwist2d(
                twist.dx * weight,
                twist.dy * weight,
                twist.dtheta * weight,
                timestamp
            ));
        pruneToRoot();
    }

    @SuppressWarnings("unchecked")
    public <T extends WheelPositions<T>> void addDriveSample(Kinematics<?, T> kinematics, T wheelPositions, double timestamp, double weight) {
        if (prevWheelPositions == null) {
            prevWheelPositions = wheelPositions;
            return;
        }
        Twist2d twist = kinematics.toTwist2d(
            (T) prevWheelPositions,
            wheelPositions
        );
        samples.add(new TimestampedTwist2d(
            twist.dx * weight,
            twist.dy * weight,
            twist.dtheta * weight,
            timestamp
        ));
        prevWheelPositions = wheelPositions;
        pruneToRoot();
    }

    public Pose2d getEstimatedPose() {
        return poseAtIndex(samples.size());
    }
}
