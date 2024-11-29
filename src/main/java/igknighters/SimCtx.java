package igknighters;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.SimArena;
import org.ironmaple.SimRobot;
import org.ironmaple.configs.GyroConfig;
import org.ironmaple.configs.MechanismConfig;
import org.ironmaple.configs.SwerveConfig;
import org.ironmaple.configs.SwerveModuleConfig;
import org.ironmaple.configs.SwerveModuleConfig.WheelCof;
import org.ironmaple.seasonspecific.Crescendo;
import org.ironmaple.utils.GearRatio;

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
    private final SimArena arena;
    private final SimRobot simRobot;

    private final Sender<NamedPositions> poseSender;
    private final Receiver<Pose2d> resetReceiver;

    private final MechanismConfig driveMotorCfg = new MechanismConfig(DCMotor.getKrakenX60Foc(1))
            .withFriction(Volts.of(kSwerve.kDriveMotor.kS))
            .withGearRatio(GearRatio.reduction(kSwerve.DRIVE_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(1.0));
    private final MechanismConfig steerMotorCfg = new MechanismConfig(DCMotor.getFalcon500Foc(1))
            .withFriction(Volts.of(1.2))
            .withGearRatio(GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
            .withNoise(0.00)
            .withRotorInertia(KilogramSquareMeters.of(0.1));
    private final SwerveModuleConfig moduleCfg = new SwerveModuleConfig(
        driveMotorCfg,
        steerMotorCfg,
        WheelCof.BLACK_NITRILE.cof,
        kSwerve.WHEEL_DIAMETER / 2.0
    );
    private final SwerveConfig swerveConfig = new SwerveConfig(
        60.0,
        6.0,
        Units.inchesToMeters(30.5),
        Units.inchesToMeters(30.5),
        kSwerve.MODULE_CHASSIS_OFFSETS,
        moduleCfg,
        GyroConfig.ofPigeon2()
    );

    public SimCtx(Localizer localizer, boolean isSim) {
        isSimulation = isSim;
        poseSender = localizer.namedPositionsSender();
        resetReceiver = localizer.poseResetsReceiver();
        if (isSimulation) {
            arena = new Crescendo.CrescendoSimArena(Seconds.of(ConstValues.PERIODIC_TIME), 5);
            simRobot = new SimRobot(arena, swerveConfig, 1);
        } else {
            arena = null;
            simRobot = null;
        }
    }

    public boolean isActive() {
        return isSimulation;
    }

    public SimArena arena () {
        return arena;
    }

    public SimRobot robot() {
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
