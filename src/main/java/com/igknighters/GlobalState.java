package com.igknighters;

import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

import org.photonvision.EstimatedRobotPose;

import com.igknighters.util.PoseHistory;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;

public class GlobalState {
    private static final ReentrantLock globalLock = new ReentrantLock();

    private static Optional<SwerveDrivePoseEstimator> localizer = Optional.empty();

    private static PoseHistory poseHistory = new PoseHistory();

    /**
     * Meant to be used by swerve to initialize the odometry system.
     * @param odometry The odometry system to initialize
     */
    public static void onceInitOdometry(SwerveDrivePoseEstimator odometry) {
        globalLock.lock();
        try {
            if (GlobalState.localizer.isPresent()) {
                DriverStation.reportError("Odometry already present", false);
                return;
            }
            GlobalState.localizer = Optional.of(odometry);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Get the robot's pose from the odometry system.
     * @return The robot's pose
     */
    public static Pose2d getLocalizedPose() {
        globalLock.lock();
        try {
            if (!localizer.isPresent()) {
                DriverStation.reportError("Odometry not present", false);
                return new Pose2d();
            }
            return localizer.get().getEstimatedPosition();
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Reset the odometry system to the given pose.
     * @param gyroRot The gyro's rotation
     * @param pose The pose to reset to
     * @param positions The positions of the swerve modules
     */
    public static void resetLocalization(Rotation2d gyroRot, Pose2d pose, SwerveModulePosition... positions) {
        globalLock.lock();
        try {
            if (!localizer.isPresent()) {
                DriverStation.reportError("Odometry not present", false);
                return;
            }
            localizer.get().resetPosition(gyroRot, positions, pose);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Pipe swerve data into the odometry system to update the robot's pose.
     * @param gyroRot The gyro's rotation
     * @param modulePositions The positions of the swerve modules
     * @return The robot's updated pose
     */
    public static Pose2d submitSwerveData(Rotation2d gyroRot, SwerveModulePosition[] modulePositions) {
        globalLock.lock();
        try {
            if (!localizer.isPresent()) {
                DriverStation.reportError("Odometry not present", false);
                return new Pose2d();
            }
            return localizer.get().update(gyroRot, modulePositions);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Pipe vision data into the odometry system to update the robot's pose.
     * @param value The vision data
     * @param trustworthyness The trustworthyness of the vision data
     */
    public static void submitVisionData(EstimatedRobotPose value, Vector<N3> trustworthyness) {
        globalLock.lock();
        try {
            if (!localizer.isPresent()) {
                DriverStation.reportError("Odometry not present", false);
                return;
            }
            localizer.get().addVisionMeasurement(
                value.estimatedPose.toPose2d(),
                value.timestampSeconds,
                trustworthyness
            );
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Add a pose to the pose history to be queried later to validate vision data.
     * @param pose The pose to add
     */
    public static void addPoseToHistory(Pose2d pose) {
        globalLock.lock();
        try {
            poseHistory.addPose(pose);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Get a pose from the pose history.
     * @param timestamp The timestamp of the pose to get
     * @return The pose at the given timestamp
     */
    public static Pose2d getPoseFromHistory(double timestamp) {
        globalLock.lock();
        try {
            return poseHistory.lookup(timestamp);
        } finally {
            globalLock.unlock();
        }
    }
}
