package com.igknighters;

import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

import com.igknighters.commands.autos.Autos;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.vision.VisionOnlyPoseEstimator;
import com.igknighters.subsystems.vision.VisionOnlyPoseEstimator.FakeWheelPositions;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEst;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class GlobalState {
    public static enum LocalizerType {
        HYBRID(1000),
        VISION(500),
        NONE(0);

        public final int priority;

        private LocalizerType(int priority) {
            this.priority = priority;
        }
    }

    private static final ReentrantLock globalLock = new ReentrantLock();

    private static LocalizerType localizerType = LocalizerType.NONE;
    private static Optional<PoseEstimator<?>> localizer = Optional.empty();

    // private static final PoseHistory poseHistory = new PoseHistory();

    private static Optional<Field2d> field = Optional.empty();

    private static boolean autoChooserCreated = false;

    private static AtomicBoolean isUnitTest = new AtomicBoolean(false);

    private GlobalState() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static void restoreDefaultState() {
        globalLock.lock();
        try {
            localizerType = LocalizerType.NONE;
            localizer = Optional.empty();
            field = Optional.empty();
            isUnitTest.set(false);
            //intentionally ignore as this is dependent on AutoBuilder state and that cannot be restored
            // autoChooserCreated = false;
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Meant to be used by swerve to initialize the odometry system.
     * 
     * @param odometry The odometry system to initialize
     */
    public static void setLocalizer(PoseEstimator<?> localizer, LocalizerType type) {
        globalLock.lock();
        try {
            if (localizerType.priority < type.priority) {
                GlobalState.localizer = Optional.of(localizer);
                localizerType = type;
            }
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Get the robot's pose from the odometry system.
     * 
     * @return The robot's pose
     */
    public static Pose2d getLocalizedPose() {
        globalLock.lock();
        try {
            if (!localizer.isPresent() || localizerType == LocalizerType.NONE) {
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
     * 
     * @param gyroRot   The gyro's rotation
     * @param pose      The pose to reset to
     * @param positions The positions of the swerve modules
     */
    public static void resetSwerveLocalization(Rotation2d gyroRot, Pose2d pose, SwerveModulePosition... positions) {
        globalLock.lock();
        try {
            if (!localizer.isPresent() || localizerType == LocalizerType.NONE) {
                DriverStation.reportError("Localizer not present", false);
                return;
            }
            if (localizerType == LocalizerType.HYBRID) {
                ((SwerveDrivePoseEstimator) localizer.get()).resetPosition(gyroRot, positions, pose);
                return;
            } else {
                DriverStation.reportError("Localizer does not support Swerve", false);
                return;
            }
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Pipe swerve data into the odometry system to update the robot's pose.
     * 
     * @param gyroRot         The gyro's rotation
     * @param modulePositions The positions of the swerve modules
     * @return The robot's updated pose
     */
    public static Pose2d submitSwerveData(Rotation2d gyroRot, SwerveModulePosition[] modulePositions) {
        globalLock.lock();
        try {
            // if (!localizer.isPresent()) {
            // DriverStation.reportError("Odometry not present", false);
            // return new Pose2d();
            // }
            // return localizer.get().update(gyroRot, modulePositions);

            if (!localizer.isPresent() || localizerType == LocalizerType.NONE) {
                DriverStation.reportError("Localizer not present", false);
                return new Pose2d();
            }
            if (localizerType == LocalizerType.HYBRID) {
                return ((SwerveDrivePoseEstimator) localizer.get()).update(gyroRot, modulePositions);
            } else {
                DriverStation.reportError("Localizer does not support Swerve", false);
                return new Pose2d();
            }
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Pipe vision data into the odometry system to update the robot's pose.
     * 
     * @param value           The vision data
     * @param trustworthyness The trustworthyness of the vision data
     */
    public static void submitVisionData(VisionPoseEst value, double ambiguity) {
        globalLock.lock();
        try {
            if (!localizer.isPresent() || localizerType == LocalizerType.NONE) {
                DriverStation.reportError("Localizer not present", false);
                return;
            }
            if (localizerType == LocalizerType.VISION) {
                ((VisionOnlyPoseEstimator) localizer.get()).addVisionMeasurement(
                        value.pose.toPose2d(),
                        value.timestamp,
                        VecBuilder.fill(1.0, 1.0, 1.0));
                var pose = ((VisionOnlyPoseEstimator) localizer.get())
                        .update(
                                value.pose.toPose2d().getRotation(),
                                new FakeWheelPositions());
                field.ifPresent(field2d -> field2d.setRobotPose(pose));
            } else if (localizerType == LocalizerType.HYBRID) {
                ((SwerveDrivePoseEstimator) localizer.get()).addVisionMeasurement(
                        value.pose.toPose2d(),
                        value.timestamp,
                        VecBuilder.fill(ambiguity, ambiguity, 0.0));
            } else {
                DriverStation.reportError("Localizer does not support Vision", false);
                return;
            }
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Create and publish the field to network tables,
     * this is also called by {@link GlobalState#modifyField(Consumer)} if the field
     * is not already published.
     */
    public static void publishField() {
        globalLock.lock();
        try {
            if (field.isPresent()) {
                DriverStation.reportError("Field already published", false);
                return;
            }
            field = Optional.of(new Field2d());
            var builder = new SendableBuilderImpl();
            builder.setTable(
                    NetworkTableInstance
                            .getDefault()
                            .getTable("Visualizers")
                            .getSubTable("Field"));
            field.get().initSendable(builder);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Run a function that modifies the field in a thread-safe manner.
     * 
     * @param modifier The function to receive the field
     */
    public static void modifyField(Consumer<Field2d> modifier) {
        globalLock.lock();
        try {
            if (!field.isPresent()) {
                publishField();
            }
            modifier.accept(field.get());
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Initialize the auto chooser.
     * This is called by {@link RobotContainer#setupAutos()}.
     * This can only be called once, further calls will log error and do nothing.
     */
    public static void onceInitAutoChooser(Swerve swerve) {
        globalLock.lock();
        try {
            if (autoChooserCreated) {
                DriverStation.reportError("Auto chooser already created", false);
                return;
            }
            autoChooserCreated = true;
            Autos.createSendableChooser(swerve);
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Get the auto command from the auto chooser.
     */
    public static Command getAutoCommand() {
        globalLock.lock();
        try {
            if (!autoChooserCreated) {
                return Commands.none().withName("Empty Auto Command");
            }
            return Autos.getAutonomousCommand();
        } finally {
            globalLock.unlock();
        }
    }

    public static void log() {
        globalLock.lock();
        try {
            localizer.ifPresent(l -> Logger.recordOutput("Global/LocalizedPose", l.getEstimatedPosition()));
            Logger.recordOutput("Global/AutoCommand", GlobalState.getAutoCommand().getName());
        } finally {
            globalLock.unlock();
        }
    }

    public static boolean isUnitTest() {
        return isUnitTest.get();
    }

    public static void setUnitTest(boolean isTest) {
        GlobalState.isUnitTest.set(isTest);
    }
}
