package org.ironmaple;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.PhysicsWorld;
import org.dyn4j.world.World;
import org.ironmaple.SimGamePiece.GamePieceVariant;
import org.ironmaple.utils.ProjectileUtil;
import org.ironmaple.utils.mathutils.GeometryConvertor;


public abstract class SimArena {
    public static final class SimulationTiming {
        public final Time period;
        public final int ticksPerPeriod;
        public final Time dt;

        private SimulationTiming(double robotPeriodSeconds, int simulationSubTicksPerPeriod) {
            this.period = Seconds.of(robotPeriodSeconds);
            this.ticksPerPeriod = simulationSubTicksPerPeriod;
            this.dt = Seconds.of(robotPeriodSeconds / ((double) ticksPerPeriod));
        }
    }

    protected final ReentrantLock worldLock = new ReentrantLock();
    protected final World<Body> physicsWorld = new World<>();
    protected final Set<SimGamePiece> gamePieces = ConcurrentHashMap.newKeySet();
    protected final Set<SimRobot> robots = ConcurrentHashMap.newKeySet();
    public final SimulationTiming timing;

    /**
     *
     *
     * <h2>Constructs a new simulation arena with the specified field map of obstacles.</h2>
     *
     * <p>This constructor initializes a physics world with zero gravity and adds the provided
     * obstacles to the world.
     *
     * <p>It also sets up the collections for drivetrain simulations, game pieces, projectiles, and
     * intake simulations.
     *
     * @param obstaclesMap the season-specific field map containing the layout of obstacles for the
     *     simulation
     * @param period the duration of each simulation period in seconds
     * @param ticksPerPeriod the number of sub-ticks to execute in each simulation period
     */
    protected SimArena(FieldMap obstaclesMap, double period, int ticksPerPeriod) {
        this.timing = new SimulationTiming(period, ticksPerPeriod);
        this.physicsWorld.setGravity(PhysicsWorld.ZERO_GRAVITY);
        for (Body obstacle : obstaclesMap.obstacles) {
            this.physicsWorld.addBody(obstacle);
        }
    }

    void withWorld(Consumer<World<Body>> worldModifier) {
        try {
            worldLock.lock();
            worldModifier.accept(physicsWorld);
        } finally {
            worldLock.unlock();
        }
    }

    /**
     *
     *
     * <h2>Registers a {@link SimGamePiece} to the Simulation.</h2>
     *
     * <p>The user will have to call one the following methods to add the game piece to the field:
     * <ul>
     *   <li>{@link SimGamePiece#place(Translation2d)}
     *   <li>{@link SimGamePiece#slide(Translation2d, Translation2d)}
     *   <li>{@link SimGamePiece#launch(Pose3d, Translation3d, ProjectileUtil.ProjectileDynamics)}
     *   <li>{@link SimGamePiece#intake(Supplier, Supplier)}
     * </ul>
     *
     * @param variant the variant of game piece to be registered
     * @return the game piece that was created and registered
     */
    public SimGamePiece createGamePiece(GamePieceVariant variant) {
        SimGamePiece gamePiece = new SimGamePiece(variant, this);
        this.gamePieces.add(gamePiece);
        return gamePiece.userControlled();
    }

    /**
     *
     *
     * <h2>Update the simulation world.</h2>
     *
     * <p>This method should be called ONCE in {@link IterativeRobotBase#simulationPeriodic()}
     */
    public void simulationPeriodic() {
        final long t0 = System.nanoTime();
        competitionPeriodic();
        // move through a few sub-periods in each update
        for (int i = 0; i < timing.ticksPerPeriod; i++) {
            simulationSubTick();
        }

        SmartDashboard.putNumber(
                "MapleArenaSimulation/Dyn4jEngineCPUTimeMS", (System.nanoTime() - t0) / 1000000.0);
    }

    private void simulationSubTick() {
        for (final SimRobot otherRobot : robots)
            otherRobot.simTick();

        for (final SimGamePiece gamePiece : gamePieces)
            gamePiece.simulationSubTick();

        withWorld(world -> world.update(timing.dt.in(Seconds)));
    }

    /**
     * Returns a stream of all game pieces tracked by the arena.
     * 
     * There are some helpful methods to better utilize the stream.
     *
     * <pre><code>
     * // Shows off getting an array of the positions of all cubes on the field
     * Pose3d[] gamePiecePositions = arena.gamePieces()
     *     .filter(GamePieceState.ON_FIELD::isInState) // Only get the game pieces that are on the field
     *     .filter(CUBE_GP::isOfVariant) // Only get the game pieces that are cubes
     *     .filter(GamePiece::isLibraryControlled) // Only get the game pieces that are controlled by the library
     *     .collect(GamePiece.poseStreamCollector()); // Collect the poses of the game pieces into an array
     * </code></pre>
     */
    public Stream<SimGamePiece> gamePieces() {
        return List.copyOf(gamePieces).stream();
    }

    /**
     *
     *
     * <h2>Resets the Field for Autonomous Mode.</h2>
     *
     * <p>This method clears all current game pieces from the field and places new game pieces in
     * their starting positions for the autonomous mode.
     */
    public void resetFieldForAuto() {
        gamePieces.forEach(gp -> gp.withLib(gp2 -> gp2.delete()));
        gamePieces.clear();
        placeGamePiecesOnField();
    }

    /**
     *
     *
     * <h2>Places Game Pieces on the Field for Autonomous Mode.</h2>
     *
     * <p>This method sets up the game pieces on the field, typically in their starting positions for
     * autonomous mode.
     *
     * <p>It should be implemented differently for each season-specific subclass of {@link
     * SimArena} to reflect the unique game piece placements for that season's game.
     */
    protected abstract void placeGamePiecesOnField();

    /**
     *
     *
     * <h2>Season-Specific Actions to Execute in {@link SimArena#simulationPeriodic()}.</h2>
     *
     * <p>This method defines season-specific tasks to be executed during the {@link
     * SimArena#simulationPeriodic()} method.
     *
     * <p>For example:
     *
     * <ul>
     *   <li>Updating the score counts.
     *   <li>Simulating human player activities.
     * </ul>
     *
     * <p>This method should be implemented in the season-specific subclass of {@link SimArena}
     * to reflect the unique aspects of that season's game.
     */
    protected abstract void competitionPeriodic();

    /**
     *
     *
     * <h1>Represents an Abstract Field Map</h1>
     *
     * <p>Stores the layout of obstacles and game pieces.
     *
     * <p>For each season-specific subclass of {@link SimArena}, there should be a corresponding
     * subclass of this class to store the field map for that specific season's game.
     */
    public abstract static class FieldMap {
        private final List<Body> obstacles = new ArrayList<>();

        protected void addBorderLine(Translation2d startingPoint, Translation2d endingPoint) {
            addCustomObstacle(
                    Geometry.createSegment(
                            GeometryConvertor.toDyn4jVector2(startingPoint),
                            GeometryConvertor.toDyn4jVector2(endingPoint)),
                    new Pose2d());
        }

        protected void addRectangularObstacle(
                double width, double height, Pose2d absolutePositionOnField) {
            addCustomObstacle(Geometry.createRectangle(width, height), absolutePositionOnField);
        }

        protected void addCustomObstacle(Convex shape, Pose2d absolutePositionOnField) {
            final Body obstacle = createObstacle(shape);

            obstacle.getTransform().set(GeometryConvertor.toDyn4jTransform(absolutePositionOnField));

            obstacles.add(obstacle);
        }

        private static Body createObstacle(Convex shape) {
            final Body obstacle = new Body();
            obstacle.setMass(MassType.INFINITE);
            final BodyFixture fixture = obstacle.addFixture(shape);
            fixture.setFriction(0.6);
            fixture.setRestitution(0.3);
            return obstacle;
        }
    }
}