package igknighters.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import sham.ShamSwerve;

import igknighters.Localizer;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.commands.swerve.teleop.TeleopSwerveBaseCmd;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import igknighters.constants.ConstValues.kSwerve;
import igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import igknighters.subsystems.swerve.control.SwerveSetpoint;
import igknighters.subsystems.swerve.control.SwerveSetpointGenerator;
import igknighters.subsystems.swerve.gyro.Gyro;
import igknighters.subsystems.swerve.gyro.GyroReal;
import igknighters.subsystems.swerve.gyro.GyroSim2;
import igknighters.subsystems.swerve.module.SwerveModule;
import igknighters.subsystems.swerve.module.SwerveModuleReal;
import igknighters.subsystems.swerve.module.SwerveModuleSim2;
import igknighters.subsystems.swerve.module.SwerveModule.AdvancedSwerveModuleState;
import igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.subsystems.swerve.odometryThread.SwerveOdometryThread;
import igknighters.util.logging.Tracer;
import igknighters.constants.ConstValues;

/**
 * This is the subsystem for our swerve drivetrain.
 * The Swerve subsystem is composed of 5 components, 4 SwerveModules and 1 Gyro.
 * The SwerveModules are the physical wheels and the Gyro is the sensor that
 * measures the robot's rotation.
 * The Swerve subsystem is responsible for controlling the SwerveModules and
 * reading the Gyro.
 * 
 * The Swerve subsystem is also responsible for updating the robot's pose and
 * submitting data to the Localizer.
 * 
 * {@link https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html}
 * 
 * The coordinate system used in this code is the field coordinate system.
 */
public class Swerve implements LockFullSubsystem {
    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();

    private final Gyro gyro;
    private final SwerveModule[] swerveMods;
    private final SwerveOdometryThread odometryThread;

    private final SwerveVisualizer visualizer;
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        kSwerve.MODULE_CHASSIS_OFFSETS,
        DCMotor.getKrakenX60Foc(1).withReduction(kSwerve.DRIVE_GEAR_RATIO),
        DCMotor.getFalcon500(1).withReduction(kSwerve.STEER_GEAR_RATIO),
        kSwerve.SLIP_CURRENT,
        65.0,
        7.0,
        kSwerve.WHEEL_DIAMETER,
        1.5,
        0.0
    );

    private final Optional<ShamSwerve> sim;

    private Optional<TeleopSwerveBaseCmd> defaultCommand = Optional.empty();
    private SwerveSetpoint setpoint = SwerveSetpoint.zeroed();

    public Swerve(final Localizer localizer, final SimCtx simCtx) {
        if (Robot.isSimulation()) {
            sim = Optional.of((ShamSwerve) simCtx.robot().getDriveTrain());
            final SimSwerveOdometryThread ot = new SimSwerveOdometryThread(250, localizer.swerveDataSender());
            swerveMods = new SwerveModule[] {
                    new SwerveModuleSim2(0, ot, sim.get()),
                    new SwerveModuleSim2(1, ot, sim.get()),
                    new SwerveModuleSim2(2, ot, sim.get()),
                    new SwerveModuleSim2(3, ot, sim.get()),
            };
            gyro = new GyroSim2(sim.get().getGyro(), ot);
            odometryThread = ot;
        } else {
            sim = Optional.empty();
            final RealSwerveOdometryThread ot = new RealSwerveOdometryThread(
                250,
                rots -> (rots / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE,
                localizer.swerveDataSender()
            );
            swerveMods = new SwerveModule[] {
                    new SwerveModuleReal(0, ot),
                    new SwerveModuleReal(1, ot),
                    new SwerveModuleReal(2, ot),
                    new SwerveModuleReal(3, ot)
            };
            gyro = new GyroReal(ot);
            odometryThread = ot;
        }

        visualizer = new SwerveVisualizer(swerveMods);

        odometryThread.start();
    }

    public void drive(ChassisSpeeds speeds) {
        log("targetChassisSpeed", speeds);

        setpoint = setpointGenerator.generateSimpleSetpoint(setpoint, speeds, ConstValues.PERIODIC_TIME);

        setModuleStates(setpoint.moduleStates());
    }

    /**
     * Offsets the gyro to define the current yaw as the supplied value
     * 
     * @param rot A {@link Rotation2d} representing the desired yaw
     */
    public void setYaw(Rotation2d rot) {
        gyro.setYawRads(rot.getRadians());
    }

    /**
     * @return The raw gyro yaw value in radians
     */
    public double getYawRads() {
        return gyro.getYawRads();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : swerveMods) {
            modulePositions[module.getModuleId()] = module.getCurrentPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(AdvancedSwerveModuleState[] desiredStates) {
        log("regurgutatedChassisSpeed", kSwerve.KINEMATICS.toChassisSpeeds(desiredStates));

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(desiredStates[module.getModuleId()]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : swerveMods) {
            states[module.getModuleId()] = module.getCurrentState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeed() {
        return kSwerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void setVoltageOut(double voltage, Rotation2d angle) {
        for (SwerveModule module : swerveMods) {
            module.setVoltageOut(voltage, angle);
        }
    }

    @Override
    public void periodic() {
        Tracer.startTrace("SwervePeriodic");

        for (SwerveModule module : swerveMods) {
            Tracer.traceFunc(module.name, module::periodic);
        }

        Tracer.traceFunc("Gyro", gyro::periodic);

        if (DriverStation.isDisabled()) {
            log("targetChassisSpeed", ZERO_SPEEDS);
        }

        log("measuredChassisSpeed", getChassisSpeed());

        visualizer.update();

        defaultCommand.ifPresent(cmd -> {
            log("Commanded", cmd);
        });

        Tracer.endTrace();
    }

    public void setDefaultCommand(TeleopSwerveBaseCmd defaultCmd) {
        defaultCommand = Optional.of(defaultCmd);
        CommandScheduler.getInstance().setDefaultCommand(this, defaultCmd);
    }
}