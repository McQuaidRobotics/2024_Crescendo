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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;

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

    private final MechanismConfig driveMotorCfg = new MechanismConfig(DCMotor.getKrakenX60Foc(1))
            .withFriction(Volts.of(kSwerve.kDriveMotor.kS))
            .withGearRatio(GearRatio.reduction(kSwerve.DRIVE_GEAR_RATIO))
            .withNoise(0.03)
            .withRotorInertia(KilogramSquareMeters.of(0.025));
    private final MechanismConfig steerMotorCfg = new MechanismConfig(DCMotor.getKrakenX60Foc(1))
            .withFriction(Volts.of(0.25))
            .withGearRatio(GearRatio.reduction(kSwerve.STEER_GEAR_RATIO))
            .withNoise(0.05)
            .withRotorInertia(KilogramSquareMeters.of(0.005));
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

    public SimCtx(boolean isSim) {
        isSimulation = isSim;
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
}
