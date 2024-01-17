package com.igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;

public class PoseHistory {
    private final ArrayList<TimestampedPose2d> prevPoses;
    private final double maxAge;

    /**
     * Creates a PoseHistory with a default max age of 0.5 seconds.
     */
    public PoseHistory() {
        this(0.5);
    }

    /**
     * Creates a PoseHistory with the given max age.
     * @param maxAgeSeconds The lifetime of a pose in the history
     */
    public PoseHistory(double maxAgeSeconds) {
        prevPoses = new ArrayList<>();
        maxAge = maxAgeSeconds;
    }

    /**
     * Adds a Pose2d to the history with a timestamp of now.
     * @param pose The pose to add
     */
    public void addPose(Pose2d pose) {
        double timestamp = Timer.getFPGATimestamp();

        prevPoses.add(new TimestampedPose2d(pose, timestamp));
        if (prevPoses.get(0).timestamp < timestamp - maxAge) {
            prevPoses.remove(0);
        }
    }

    /**
     * Returns the Pose2d with the closest timestamp to the given timestamp.
     * @param timestamp The timestamp to lookup
     * @return The closest pose
     */
    public Pose2d lookup(double timestamp) {
        TimestampedPose2d closest = prevPoses.get(prevPoses.size() - 1);

        for (int i = prevPoses.size() - 2; i >= 0; i--) {
            if (Math.abs(prevPoses.get(i).timestamp - timestamp) < Math.abs(closest.timestamp - timestamp)) {
                closest = prevPoses.get(i);
            } else {
                break;
            }
        }

        return closest.pose;
    }

    private static class TimestampedPose2d {
        private final Pose2d pose;
        private final double timestamp;

        private TimestampedPose2d(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }
}
