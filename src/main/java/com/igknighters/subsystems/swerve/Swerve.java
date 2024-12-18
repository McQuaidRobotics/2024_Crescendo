package com.igknighters.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBaseCmd;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import com.igknighters.subsystems.swerve.gyro.Gyro;
import com.igknighters.subsystems.swerve.gyro.GyroReal;
import com.igknighters.subsystems.swerve.gyro.GyroSim;
import com.igknighters.subsystems.swerve.module.SwerveModule;
import com.igknighters.subsystems.swerve.module.SwerveModuleReal;
import com.igknighters.subsystems.swerve.module.SwerveModuleSim;
import com.igknighters.subsystems.swerve.odometryThread.RealSwerveOdometryThread;
import com.igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import com.igknighters.subsystems.swerve.odometryThread.SwerveOdometryThread;
import com.igknighters.util.logging.Tracer;
import com.igknighters.util.plumbing.Channel.Sender;
import com.igknighters.constants.ConstValues;

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
    private final SwerveSetpointProcessor setpointProcessor = new SwerveSetpointProcessor();

    private final Sender<ChassisSpeeds> velocitySender;

    private Optional<TeleopSwerveBaseCmd> defaultCommand = Optional.empty();

    public Swerve(final Localizer localizer) {
        if (Robot.isReal()) {
            RealSwerveOdometryThread ot = new RealSwerveOdometryThread(
                250,
                rots -> (rots / kSwerve.DRIVE_GEAR_RATIO) * kSwerve.WHEEL_CIRCUMFERENCE,
                localizer.swerveDataSender()
            );
            swerveMods = new SwerveModule[] {
                    new SwerveModuleReal(ConstValues.kSwerve.kMod0.CONSTANTS, true, ot),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod1.CONSTANTS, true, ot),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod2.CONSTANTS, true, ot),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod3.CONSTANTS, true, ot)
            };
            gyro = new GyroReal(ot);
            odometryThread = ot;
        } else {
            SimSwerveOdometryThread ot = new SimSwerveOdometryThread(250, localizer.swerveDataSender());
            swerveMods = new SwerveModule[] {
                    new SwerveModuleSim(ConstValues.kSwerve.kMod0.CONSTANTS, ot),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod1.CONSTANTS, ot),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod2.CONSTANTS, ot),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod3.CONSTANTS, ot)
            };
            gyro = new GyroSim(this::getChassisSpeed, ot);
            odometryThread = ot;
        }

        visualizer = new SwerveVisualizer(swerveMods);

        setpointProcessor.setDisabled(true);

        odometryThread.start();

        velocitySender = localizer.velocityChannel().sender();
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        log("targetChassisSpeed", speeds);

        setModuleStates(
            kSwerve.KINEMATICS.toSwerveModuleStates(speeds),
                isOpenLoop);
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
            modulePositions[module.getModuleNumber()] = module.getCurrentPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        log("regurgutatedChassisSpeed", kSwerve.KINEMATICS.toChassisSpeeds(desiredStates));

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(desiredStates[module.getModuleNumber()], isOpenLoop);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : swerveMods) {
            states[module.getModuleNumber()] = module.getCurrentState();
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
            Tracer.traceFunc("SwerveModule[" + module.getModuleNumber() + "]", module::periodic);
        }

        Tracer.traceFunc("Gyro", gyro::periodic);

        if (DriverStation.isDisabled()) {
            log("targetChassisSpeed", ZERO_SPEEDS);
        }

        ChassisSpeeds measuredSpeeds = getChassisSpeed();
        log("measuredChassisSpeed", measuredSpeeds);
        velocitySender.send(measuredSpeeds);

        visualizer.update();

        defaultCommand.ifPresent(cmd -> {
            log("Commanded", cmd);
        });

        Tracer.endTrace();
    }

    public static double scope0To360(double angle) {
        if (angle < 0) {
            angle = 360 - (Math.abs(angle) % 360);
        } else {
            angle %= 360;
        }
        return angle;
    }

    public void setDefaultCommand(TeleopSwerveBaseCmd defaultCmd) {
        defaultCommand = Optional.of(defaultCmd);
        CommandScheduler.getInstance().setDefaultCommand(this, defaultCmd);
    }
}
