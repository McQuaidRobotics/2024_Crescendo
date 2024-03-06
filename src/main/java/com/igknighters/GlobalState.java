package com.igknighters;

import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.concurrent.atomic.AtomicBoolean;

import com.igknighters.commands.autos.Autos;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.vision.VisionOnlyPoseEstimator;
import com.igknighters.subsystems.vision.VisionOnlyPoseEstimator.FakeWheelPositions;
import com.igknighters.subsystems.vision.camera.Camera.VisionPoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import monologue.MonologueDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class GlobalState {
    public static enum LocalizerType {
        Hybrid(1000),
        Vision(500),
        None(0);

        public final int priority;

        private LocalizerType(int priority) {
            this.priority = priority;
        }
    }

    private static AtomicBoolean climbing = new AtomicBoolean(false);

    private static final ReentrantLock globalLock = new ReentrantLock();

    private static LocalizerType localizerType = LocalizerType.None;
    private static Optional<PoseEstimator<?>> localizer = Optional.empty();
    private static ChassisSpeeds velocity = new ChassisSpeeds();

    private static Optional<Field2d> field = Optional.empty();

    private static boolean autoChooserCreated = false;

    private static AtomicBoolean isUnitTest = new AtomicBoolean(false);

    private static Supplier<Rotation3d> rotSupplier = Rotation3d::new;

    private GlobalState() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static void restoreDefaultState() {
        globalLock.lock();
        try {
            localizerType = LocalizerType.None;
            localizer = Optional.empty();
            field = Optional.empty();
            isUnitTest.set(false);
            rotSupplier = Rotation3d::new;
            velocity = new ChassisSpeeds();
            // intentionally ignore as this is dependent on AutoBuilder state and that
            // cannot be restored
            // autoChooserCreated = false;
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Set the gyro rotation supplier.
     * 
     * @param rotSup
     */
    public static void setGyroRotSupplier(Supplier<Rotation3d> rotSup) {
        globalLock.lock();
        try {
            rotSupplier = rotSup;
        } finally {
            globalLock.lock();
        }
    }

    /**
     * Get the gyro rotation.
     * 
     * @return Rotation3d containing the gyro's rotation.
     */
    public static Rotation3d getGyroRot() {
        globalLock.lock();
        try {
            return rotSupplier.get();
        } finally {
            globalLock.lock();
        }
    }

    public static void setVelocity(ChassisSpeeds velo) {
        globalLock.lock();
        try {
            velocity = velo;
        } finally {
            globalLock.unlock();
        }
    }

    public static ChassisSpeeds getVelocity() {
        globalLock.lock();
        try {
            return velocity;
        } finally {
            globalLock.unlock();
        }
    }

    public static ChassisSpeeds getFieldRelativeVelocity() {
        globalLock.lock();
        try {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    velocity,
                    getGyroRot().toRotation2d());
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
            if (!localizer.isPresent() || localizerType == LocalizerType.None) {
                DriverStation.reportError("[GlobalState] Odometry not present", true);
                return new Pose2d();
            }
            return localizer.get().getEstimatedPosition();
        } finally {
            globalLock.unlock();
        }
    }

    public static Pose3d getLocalizedPose3d() {
        globalLock.lock();
        try {
            if (!localizer.isPresent() || localizerType == LocalizerType.None) {
                DriverStation.reportError("[GlobalState] Odometry not present", true);
                return new Pose3d();
            }
            if (Robot.isSimulation()) {
                return new Pose3d(getLocalizedPose());
            } else {
                Translation2d translation = getLocalizedPose().getTranslation();
                return new Pose3d(
                        new Translation3d(translation.getX(), translation.getY(), 0),
                        rotSupplier.get());
            }
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
            if (!localizer.isPresent() || localizerType == LocalizerType.None) {
                DriverStation.reportError("[GlobalState] Localizer not present", true);
                return;
            }
            if (localizerType == LocalizerType.Hybrid) {
                ((SwerveDrivePoseEstimator) localizer.get()).resetPosition(gyroRot, positions, pose);
                return;
            } else {
                DriverStation.reportError("[GlobalState] Localizer does not support Swerve", true);
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
            // DriverStation.reportError("[GlobalState] Odometry not present", false);
            // return new Pose2d();
            // }
            // return localizer.get().update(gyroRot, modulePositions);

            if (!localizer.isPresent() || localizerType == LocalizerType.None) {
                DriverStation.reportError("[GlobalState] Localizer not present", true);
                return new Pose2d();
            }
            if (localizerType == LocalizerType.Hybrid) {
                return ((SwerveDrivePoseEstimator) localizer.get()).update(gyroRot, modulePositions);
            } else {
                DriverStation.reportError("[GlobalState] Localizer does not support Swerve", true);
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
    public static void submitVisionData(VisionPoseEstimate value, double ambiguity) {
        globalLock.lock();
        try {
            if (!localizer.isPresent() || localizerType == LocalizerType.None) {
                DriverStation.reportError("[GlobalState] Localizer not present", true);
                return;
            }
            if (localizerType == LocalizerType.Vision) {
                ((VisionOnlyPoseEstimator) localizer.get()).addVisionMeasurement(
                        value.pose().toPose2d(),
                        value.timestamp(),
                        VecBuilder.fill(1.0, 1.0, 1.0));
                var pose = ((VisionOnlyPoseEstimator) localizer.get())
                        .update(
                                value.pose().toPose2d().getRotation(),
                                new FakeWheelPositions());
                field.ifPresent(field2d -> field2d.setRobotPose(pose));
            } else if (localizerType == LocalizerType.Hybrid) {
                ((SwerveDrivePoseEstimator) localizer.get()).addVisionMeasurement(
                        new Pose2d(
                                value.pose().getTranslation().toTranslation2d(),
                                GlobalState.rotSupplier.get().toRotation2d()),
                        value.timestamp(),
                        VecBuilder.fill(ambiguity, ambiguity, 1.0));
            } else {
                DriverStation.reportError("[GlobalState] Localizer does not support Vision", true);
                return;
            }
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * Create and publish the field to network tables,
     * this is also called by {@link GlobalState#modifyField2d(Consumer)} if the
     * field
     * is not already published.
     */
    public static void publishField2d() {
        globalLock.lock();
        try {
            if (field.isPresent()) {
                DriverStation.reportError("[GlobalState] Field already published", true);
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
    public static void modifyField2d(Consumer<Field2d> modifier) {
        globalLock.lock();
        try {
            if (!field.isPresent()) {
                publishField2d();
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
                DriverStation.reportError("[GlobalState] Auto chooser already created", true);
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
            localizer.ifPresent(
                    l -> MonologueDashboard.put("Global/LocalizedPose", l.getEstimatedPosition().toString()));
            MonologueDashboard.put("Global/AutoCommand", GlobalState.getAutoCommand().getName());
        } finally {
            globalLock.unlock();
        }
    }

    /**
     * @return If the code is being run as part of a unit test
     */
    public static boolean isUnitTest() {
        return isUnitTest.get();
    }

    /**
     * Declare if the code is being run as part of a unit test
     * 
     * @param isTest If the code should be run as a unit test
     */
    public static void setUnitTest(boolean isTest) {
        GlobalState.isUnitTest.set(isTest);
    }

    /**
     * @return If the robot is currently in a climbing mode
     */
    public static boolean isClimbing() {
        return GlobalState.climbing.get();
    }

    /**
     * Declare if the robot is in a climbing mode
     * 
     * @param isClimbing Whether the robot is climbing or not
     */
    public static void setClimbing(boolean isClimbing) {
        GlobalState.climbing.set(isClimbing);
    }
}
