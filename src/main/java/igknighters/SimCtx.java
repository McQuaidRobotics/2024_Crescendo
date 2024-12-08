package igknighters;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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
import igknighters.constants.ConstValues.kSwerve;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;

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

    private final Sender<NamedPositions> poseSender;
    private final Receiver<Pose2d> resetReceiver;

    private final ShamMechanismConfig driveMotorCfg = new ShamMechanismConfig(DCMotor.getKrakenX60Foc(1))
            .withFriction(Volts.of(kSwerve.kDriveMotor.kS))
            .withGearRatio(GearRatio.reduction(kSwerve.DRIVE_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(1.0));
    private final ShamMechanismConfig steerMotorCfg = new ShamMechanismConfig(DCMotor.getFalcon500Foc(1))
            .withFriction(Volts.of(1.2))
            .withGearRatio(GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(0.1));
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
            simRobot = new ShamRobot<>(arena, swerveConfig, 1);
        } else {
            arena = null;
            simRobot = null;
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

    public void update() {
        if (isSimulation) {
            if (resetReceiver.hasData()) {
                final var poses = resetReceiver.recvAll();
                robot().getDriveTrain().setChassisWorldPose(poses[poses.length - 1], true);
            }
            arena.simulationPeriodic();
            poseSender.send(new NamedPositions("SimRobot", simRobot.getDriveTrain().getChassisWorldPose()));
        }
    }
}
