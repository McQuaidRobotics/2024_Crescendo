package com.igknighters.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.gyro.Gyro;
import com.igknighters.subsystems.swerve.gyro.GyroReal;
import com.igknighters.subsystems.swerve.gyro.GyroSim;
import com.igknighters.subsystems.swerve.module.SwerveModule;
import com.igknighters.subsystems.swerve.module.SwerveModuleReal;
import com.igknighters.subsystems.swerve.module.SwerveModuleSim;
import com.igknighters.util.CANBusLogging;
import com.igknighters.util.Tracer;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;

/**
 * This is the subsystem for our swerve drivetrain.
 * The Swerve subsystem is composed of 5 components, 4 SwerveModules and 1 Gyro.
 * The SwerveModules are the physical wheels and the Gyro is the sensor that
 * measures the robot's rotation.
 * The Swerve subsystem is responsible for controlling the SwerveModules and
 * reading the Gyro.
 * 
 * The Swerve subsystem is also responsible for updating the robot's pose and
 * submitting data to the GlobalState.
 * 
 * {@link https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html}
 * 
 * The coordinate system used in this code is the field coordinate system.
 */
public class Swerve extends SubsystemBase implements Logged {
    private final Gyro gyro;
    private final SwerveModule[] swerveMods;
    private final SwerveVisualizer visualizer;
    private final SwerveSetpointProcessor setpointProcessor = new SwerveSetpointProcessor();

    private RotationalController rotController = new RotationalController();

    public Swerve() {

        if (RobotBase.isReal()) {
            swerveMods = new SwerveModule[] {
                    new SwerveModuleReal(ConstValues.kSwerve.kMod0.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod1.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod2.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.kMod3.CONSTANTS, false)
            };
            gyro = new GyroReal();
        } else {
            swerveMods = new SwerveModule[] {
                    new SwerveModuleSim(ConstValues.kSwerve.kMod0.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod1.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod2.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.kMod3.CONSTANTS)
            };
            gyro = new GyroSim(this::getChassisSpeed);
        }

        GlobalState.setLocalizer(
                new SwerveDrivePoseEstimator(
                        kSwerve.SWERVE_KINEMATICS,
                        getYawWrappedRot(),
                        getModulePositions(),
                        new Pose2d(
                                new Translation2d(
                                        FieldConstants.FIELD_LENGTH / 2.0,
                                        FieldConstants.FIELD_WIDTH / 2.0),
                                new Rotation2d())),
                GlobalState.LocalizerType.Hybrid);

        visualizer = new SwerveVisualizer(this, swerveMods);

        setpointProcessor.setDisabled(true);

        CANBusLogging.logBus(ConstValues.kSwerve.CANBUS);
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        log("targetChassisSpeed", speeds);

        setModuleStates(
                kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds),
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
     * @return The gyro yaw value in degrees, wrapped to 0-360, as a Rotation2d
     */
    public Rotation2d getYawWrappedRot() {
        return Rotation2d.fromDegrees(
                scope0To360(
                        Units.radiansToDegrees(this.getYawRads())));
    }

    /**
     * @return The raw gyro yaw value in radians
     */
    public double getYawRads() {
        return gyro.getYawRads();
    }

    /**
     * @return The raw gyro pitch value in radians
     */
    public double getPitchRads() {
        return gyro.getPitchRads();
    }

    /**
     * @return The raw gyro roll value in radians
     */
    public double getRollRads() {
        return gyro.getRollRads();
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
        return kSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return GlobalState.getLocalizedPose();
    }

    public void resetOdometry(Pose2d pose) {
        GlobalState.resetSwerveLocalization(getYawWrappedRot(), pose, getModulePositions());
    }

    public double rotVeloForRotation(Rotation2d wantedAngle) {
        double targetAngleRads = wantedAngle.getRadians();
        double currentAngleRads = getYawRads();

        log("WantedAngle", targetAngleRads);
        log("CurrentAngle", currentAngleRads);

        return rotController.calculate(currentAngleRads, targetAngleRads);
    }

    public void resetRotController() {
        rotController.reset(MathUtil.angleModulus(getYawRads()), getChassisSpeed().omegaRadiansPerSecond);
    }

    public Rotation2d rotationRelativeToPose(Rotation2d wantedAngleOffet, Translation2d pose) {
        Translation2d currentTrans = getPose().getTranslation();
        double angleBetween = Math.atan2(
                pose.getY() - currentTrans.getY(),
                pose.getX() - currentTrans.getX());
        return Rotation2d.fromRadians(angleBetween)
                .plus(wantedAngleOffet);
    }

    @Override
    public void periodic() {
        Tracer.startTrace("SwervePeriodic");

        for (SwerveModule module : swerveMods) {
            Tracer.traceFunc("SwerveModule[" + module.getModuleNumber() + "]", module::periodic);
        }

        Tracer.traceFunc("Gyro", gyro::periodic);

        visualizer.update(GlobalState.submitSwerveData(getYawWrappedRot(), getModulePositions()));

        if (DriverStation.isDisabled()) {
            log("targetChassisSpeed", new ChassisSpeeds());
        }

        GlobalState.setVelocity(getChassisSpeed());

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
}
