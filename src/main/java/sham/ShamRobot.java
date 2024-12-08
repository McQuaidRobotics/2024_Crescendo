package sham;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.ConcurrentLinkedQueue;

import sham.ShamArena.ShamEnvTiming;
import sham.ShamGamePiece.GamePieceVariant;
import sham.configs.ShamDriveTrainConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.geometry.Rectangle2d;

/**
 * Represents a robot in the sham environment.
 * 
 * A robot is composed of:
 * <ul>
 * <li> A {@link ShamDriveTrain} subclass that represents the robot's drivetrain.
 * <li> A {@link ShamIndexer} object that stores {@link ShamGamePiece}s for the robot.
 * <li> A {@link ShamBattery} object that simulates the robot's battery.
 * <li> A collection of {@link ShamIntake} objects that represent the robot's intakes.
 * <li> A collection of {@link ShamMechanism} objects that represent the robot's mechanisms.
 * </ul>
 */
public class ShamRobot {
    private final ShamArena arena;
    private final ShamDriveTrain driveTrain;
    // may move towards multi indexers soon
    private final ShamIndexer gamePieceStorage;
    private final ShamBattery battery = new ShamBattery();
    private final ConcurrentLinkedQueue<ShamIntake> intakes = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<ShamMechanism> mechanisms = new ConcurrentLinkedQueue<>();

    public <T extends ShamDriveTrain, C extends ShamDriveTrainConfig<T, C>> ShamRobot(ShamArena arena, C drivetrainConfig, int gamePieceStorageCapacity) {
        this.arena = arena;
        arena.robots.add(this);
        this.driveTrain = ShamDriveTrain.createDriveTrainSimulation(this, drivetrainConfig);
        arena.withWorld(world -> world.addBody(driveTrain.chassis));
        this.gamePieceStorage = new ShamIndexer(gamePieceStorageCapacity);
    }

    void simTick() {
        driveTrain.simTick();
        // final Voltage batVolts = battery.getBatteryVoltage();
        for (var mechanism : mechanisms) {
            mechanism.update(Volts.of(12.0));
        }
    }

    /**
     * Returns an object that stores the timing data for the simulation.
     * 
     * @return an object that stores the timing data for the simulation.
     */
    public ShamEnvTiming timing() {
        return arena.timing;
    }

    /**
     * Creates a new intake for the robot and adds it to the simulation.
     * 
     * @param boundingBox the bounding box of the intake in the simulation world,
     *      uses robot coordinates space.
     * @param acceptedGamePieceVariants the types of game pieces that the intake can accept,
     *     if no types are provided, the intake will accept all types of game pieces.
     * @return the newly created intake.
     */
    public ShamIntake createIntake(Rectangle2d boundingBox, GamePieceVariant... acceptedGamePieceVariants) {
        var intake = new ShamIntake(
            driveTrain,
            gamePieceStorage,
            GeometryConvertor.toDyn4jRectangle(boundingBox),
            acceptedGamePieceVariants
        );
        intakes.add(intake);
        arena.withWorld(world -> world.addContactListener(intake.getGamePieceContactListener()));
        RuntimeLog.debug("Created IntakeSimulation");
        return intake;
    }

    /**
     * Adds a {@link ShamMechanism} to the robot.
     * 
     * @param mechanism the {@link ShamMechanism} to add to the robot.
     */
    public void addMechanism(ShamMechanism mechanism) {
        mechanisms.add(mechanism);
        battery.addMechanism(mechanism);
        RuntimeLog.debug("Added SimMechanism to SimRobot");
    }

    /**
     * Returns the {@link ShamDriveTrain} subclass correlated
     * with the {@link ShamDriveTrainConfig} used to create this {@link ShamRobot}.
     * 
     * If you need the specific type of the {@link ShamDriveTrain} subclass, you
     * have to cast the return value to the specific subclass.
     * 
     * @return the {@link ShamDriveTrain} subclass correlated with the
     *        {@link ShamDriveTrainConfig} used to create this {@link ShamRobot}.
     */
    public ShamDriveTrain getDriveTrain() {
        return driveTrain;
    }

    /**
     * Returns the {@link ShamIndexer} object that is used to
     * store {@link ShamGamePiece}s for this {@link ShamRobot}.
     * 
     * @return the {@link ShamIndexer} object that is used to store {@link ShamGamePiece}s.
     */
    public ShamIndexer getGamePieceStorage() {
        return gamePieceStorage;
    }
}
