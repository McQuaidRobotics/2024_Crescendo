package igknighters;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import sham.ShamArena;
import sham.ShamSwerve;
import sham.ShamRobot;
import sham.configs.ShamGyroConfig;
import sham.configs.ShamMechanismConfig;
import sham.configs.ShamSwerveConfig;
import sham.configs.ShamSwerveModuleConfig;
import sham.configs.ShamSwerveModuleConfig.WheelCof;
import sham.seasonspecific.Crescendo;
import sham.utils.GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import igknighters.Localizer.NamedPositions;
import igknighters.constants.ConstValues;
import igknighters.constants.FieldConstants;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;
import monologue.LogSink;
import monologue.Monologue;

/**
 * An object containing sim-specific objects and configurations.
 * 
 * This object should be treated similar to an optional,
 * where the simulation context is only usable if {@link #isActive()} returns true.
 */
public class SimCtx {
    private final boolean isSimulation;

    // all fields below this point will be null if isSimulation is false
    private final ShamArena arena;
    private final ShamRobot<ShamSwerve> simRobot;

    private final VisionSystemSim aprilTagSim;
    private final VisionSystemSim objectDetectionSim;
    private final TargetModel gpTargetModel = new TargetModel(
        Units.inchesToMeters(14.0),
        Units.inchesToMeters(14.0),
        Units.inchesToMeters(2.0)
    );

    private final Sender<NamedPositions> poseSender;
    private final Receiver<Pose2d> resetReceiver;

    private final ShamMechanismConfig driveMotorCfg = new ShamMechanismConfig(DCMotor.getKrakenX60Foc(1))
            .withFriction(Volts.of(0.2), Volts.of(0.175))
            .withGearRatio(GearRatio.reduction(kSwerve.DRIVE_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(0.003));
    private final ShamMechanismConfig steerMotorCfg = new ShamMechanismConfig(DCMotor.getFalcon500Foc(1))
            .withFriction(Volts.of(1.0), Volts.of(1.0))
            .withGearRatio(GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(0.02));
    private final ShamSwerveModuleConfig moduleCfg = new ShamSwerveModuleConfig(
        driveMotorCfg,
        steerMotorCfg,
        WheelCof.BLACK_NITRILE.cof,
        kSwerve.WHEEL_DIAMETER / 2.0
    );
    private final ShamSwerveConfig swerveConfig = new ShamSwerveConfig(
        60.0,
        6.0,
        Units.inchesToMeters(30.5),
        Units.inchesToMeters(30.5),
        kSwerve.MODULE_CHASSIS_OFFSETS,
        moduleCfg,
        ShamGyroConfig.ofPigeon2()
    );

    public SimCtx(Localizer localizer, boolean isSim) {
        isSimulation = isSim;
        poseSender = localizer.namedPositionsSender();
        resetReceiver = localizer.poseResetsReceiver();
        if (isSimulation) {
            arena = new Crescendo.CrescendoSimArena(Seconds.of(ConstValues.PERIODIC_TIME), 5);
            simRobot = new ShamRobot<>(arena, "User", swerveConfig, 1);
            aprilTagSim = new VisionSystemSim("AprilTags");
            aprilTagSim.addAprilTags(FieldConstants.APRIL_TAG_FIELD);
            objectDetectionSim = new VisionSystemSim("ObjectDetection");
            Monologue.publishSendable("visionSimField", aprilTagSim.getDebugField(), LogSink.NT);
        } else {
            arena = null;
            simRobot = null;
            aprilTagSim = null;
            objectDetectionSim = null;
        }
    }

    public boolean isActive() {
        return isSimulation;
    }

    public ShamArena arena () {
        return arena;
    }

    public ShamRobot<ShamSwerve> robot() {
        return simRobot;
    }

    public VisionSystemSim aprilTagSim() {
        return aprilTagSim;
    }

    public VisionSystemSim objectDetectionSim() {
        return objectDetectionSim;
    }

    public void update() {
        if (isSimulation) {
            if (resetReceiver.hasData()) {
                final var poses = resetReceiver.recvAll();
                robot().getDriveTrain().setChassisWorldPose(poses[poses.length - 1], true);
            }
            arena.simulationPeriodic();
            final Pose2d robotPose = simRobot.getDriveTrain().getChassisWorldPose();
            poseSender.send(new NamedPositions("SimRobot", robotPose));
            aprilTagSim.update(robotPose);
            objectDetectionSim.update(robotPose);
            objectDetectionSim.clearVisionTargets();
            final var objectTargets = arena.gamePieces()
                .map(gp -> new VisionTargetSim(gp.pose(), gpTargetModel))
                .toArray(VisionTargetSim[]::new);
            objectDetectionSim.addVisionTargets("gamepieces", objectTargets);
        }
    }
}
