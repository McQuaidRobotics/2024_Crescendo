package sham;

import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import monologue.procstruct.ProceduralStructGenerator;

import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.Mass;
import org.dyn4j.geometry.Vector2;
import sham.configs.ShamDriveTrainConfig;
import sham.configs.ShamSwerveConfig;
import sham.utils.FrcBody;
import sham.utils.mathutils.GeometryConvertor;

/**
 *
 *
 * <h1>Represents an Abstract Drivetrain Simulation.</h1>
 *
 * <h3>Simulates the Mass, Collision Space, and Friction of the Drivetrain.</h3>
 *
 * <p>This class models the physical properties of a drivetrain, including mass and collision space.
 *
 * <p>It also provides APIs to obtain the status (position, velocity etc.) in WPILib geometry classes.
 */
public class ShamDriveTrain {
    /**
     * https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction
     */
    public static final double kBumperCoF = 0.65;
    /**
     * https://simple.wikipedia.org/wiki/Coefficient_of_restitution
     */
    public static final double kBumperCoR = 0.005;

    protected final DataLogger logger;
    protected final FrcBody chassis = new FrcBody();

    /**
     *
     *
     * <h2>Creates a Simulation of a Drivetrain.</h2>
     *
     * <h3>Sets Up the Collision Space and Mass of the Chassis.</h3>
     *
     * <p>Since this is an abstract class, the constructor must be called from a subclass.
     *
     * <p>Note that the chassis does not appear on the simulation field upon creation. Refer to
     * {@link sham.ShamArena} for instructions on how to add it to the simulation world.
     *
     * @param config a {@link ShamDriveTrainConfig} instance containing the configurations of this drivetrain
     * @param initialPoseOnField the initial pose of the drivetrain in the simulation world
     */
    @SuppressWarnings("unchecked")
    protected ShamDriveTrain(DataLogger logger, ShamDriveTrainConfig<?, ?> config) {
        this.logger = logger;
        logger.log("config", config, (Struct<ShamDriveTrainConfig<?, ?>>) ProceduralStructGenerator
                .extractClassStructDynamic(config.getClass()).get());
        chassis.addFixture(
                Geometry.createRectangle(config.bumperLengthXMeters, config.bumperWidthYMeters),
                0.0, // zero density; mass is set explicitly
                kBumperCoF,
                kBumperCoR);

        chassis.setMass(new Mass(new Vector2(), config.robotMassKg, config.robotMoI));
    }

    /**
     * Sets the Robot's Current Pose in the Simulation World.
     *
     * <p>This method instantly teleports the robot to the specified pose in the simulation world. The robot does not
     * drive to the new pose; it is moved directly.
     *
     * @param robotPose the desired robot pose, represented as a {@link Pose2d}
     * @param resetVelocity whether to reset the robot's velocity to zero after teleporting
     */
    public void setChassisWorldPose(Pose2d robotPose, boolean resetVelocity) {
        chassis.setTransform(GeometryConvertor.toDyn4jTransform(robotPose));
        if (resetVelocity) {
            chassis.setLinearVelocity(0, 0);
            chassis.setAngularVelocity(0);
        }
    }

    /**
     * Sets the chassis's Speeds to the Given ChassisSpeeds.
     *
     * <p>The robot does not accelerate smoothly to these speeds; instead,
     * it jumps to the velocity Instantaneously.
     *
     * @param givenSpeeds the desired field-relative {@link ChassisSpeeds}
     */
    public void setChassisWorldSpeeds(ChassisSpeeds givenSpeeds) {
        chassis.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(givenSpeeds));
        chassis.setAngularVelocity(givenSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Gets the Actual Pose of the Drivetrain in the Simulation World.
     *
     * <p><strong>Note:</strong> Do not use this method to simulate odometry! For a more realistic odometry simulation,
     * use a {@link ShamSwerve} together with a
     * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
     *
     * @return a {@link Pose2d} object yielding the current world pose of the robot in the simulation
     */
    public Pose2d getChassisWorldPose() {
        return GeometryConvertor.toWpilibPose2d(chassis.getTransform());
    }

    /**
     * Gets the Actual Field-Relative Chassis Speeds from the Simulation World.
     *
     * @return the actual chassis speeds in the simulation world, <strong>Field-Relative</strong>
     */
    public ChassisSpeeds getChassisWorldSpeeds() {
        return GeometryConvertor.toWpilibChassisSpeeds(chassis.getLinearVelocity(), chassis.getAngularVelocity());
    }

    /**
     * Abstract Simulation Sub-Tick Method.
     *
     * <p>This method is called every time the simulation world is updated.
     *
     * <p>It is implemented in the sub-classes of {@link ShamDriveTrain}.
     *
     * <p>It is responsible for applying the propelling forces to the robot during each sub-tick of the simulation.
     */
    protected void simTick() {
        logger.log("chassis", chassis.snapshot(), FrcBody.FrcBodySnapshot.struct);
    }

    /**
     * Creates a drivetrain simulation with the given configuration and initial pose on the field.
     * 
     * @param <T> the type of the drivetrain simulation
     * @param <C> the type of the drivetrain simulation configuration
     * @param config the configuration of the drivetrain simulation
     * @param initialPoseOnField the initial pose of the drivetrain on the field
     * @return the created drivetrain simulation
     */
    @SuppressWarnings("unchecked")
    public static <T extends ShamDriveTrain, C extends ShamDriveTrainConfig<T, C>> T createDriveTrain(
            ShamRobot<T> robot, C config) {
        // Don't forget to update this method when adding new drivetrain configurations
        if (config instanceof ShamSwerveConfig) {
            return (T) new ShamSwerve((ShamRobot<ShamSwerve>) robot, (ShamSwerveConfig) config);
        }
        throw new IllegalArgumentException("Unknown drivetrain configuration");
    }
}
