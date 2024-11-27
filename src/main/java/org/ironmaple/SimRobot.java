package org.ironmaple;

import java.util.concurrent.ConcurrentLinkedQueue;

import org.ironmaple.SimArena.SimulationTiming;
import org.ironmaple.SimGamePiece.GamePieceVariant;
import org.ironmaple.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.GeometryConvertor;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.units.measure.Voltage;

public class SimRobot {
    private final SimArena arena;
    private final SimDriveTrain driveTrain;
    private final SimIndexer gamePieceStorage;
    private final SimBattery battery = new SimBattery();
    private final ConcurrentLinkedQueue<SimIntake> intakes = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<SimMechanism> mechanisms = new ConcurrentLinkedQueue<>();

    public <T extends SimDriveTrain, C extends DriveTrainSimulationConfig<T, C>> SimRobot(SimArena arena, C drivetrainConfig, int gamePieceStorageCapacity) {
        this.arena = arena;
        this.driveTrain = SimDriveTrain.createDriveTrainSimulation(this, drivetrainConfig);
        arena.withWorld(world -> world.addBody(driveTrain.chassis));
        this.gamePieceStorage = new SimIndexer(gamePieceStorageCapacity);
    }

    void simTick() {
        driveTrain.simTick();
        final Voltage batVolts = battery.getBatteryVoltage();
        for (var mechanism : mechanisms) {
            mechanism.update(batVolts);
        }
    }

    /**
     * Returns an object that stores the timing data for the simulation.
     * 
     * @return an object that stores the timing data for the simulation.
     */
    public SimulationTiming timing() {
        return arena.timing;
    }

    public SimIntake createIntake(Rectangle2d boundingBox, GamePieceVariant... acceptedGamePieceVariants) {
        var intake = new SimIntake(
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

    public void addMechanism(SimMechanism motor) {
        mechanisms.add(motor);
        battery.addMechanism(motor);
        RuntimeLog.debug("Added SimMechanism to SimRobot");
    }

    /**
     * Returns the {@link SimDriveTrain} subclass correlated
     * with the {@link DriveTrainSimulationConfig} used to create this {@link SimRobot}.
     * 
     * If you need the specific type of the {@link SimDriveTrain} subclass, you
     * have to cast the return value to the specific subclass.
     * 
     * @return the {@link SimDriveTrain} subclass correlated with the
     *        {@link DriveTrainSimulationConfig} used to create this {@link SimRobot}.
     */
    public SimDriveTrain getDriveTrain() {
        return driveTrain;
    }

    /**
     * Returns the {@link SimIndexer} object that is used to
     * store {@link SimGamePiece}s for this {@link SimRobot}.
     * 
     * @return the {@link SimIndexer} object that is used to store {@link SimGamePiece}s.
     */
    public SimIndexer getGamePieceStorage() {
        return gamePieceStorage;
    }
}
