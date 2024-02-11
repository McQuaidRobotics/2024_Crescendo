package com.igknighters.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

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

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.gyro.Gyro;
import com.igknighters.subsystems.swerve.gyro.GyroReal;
import com.igknighters.subsystems.swerve.gyro.GyroSim;
import com.igknighters.subsystems.swerve.module.SwerveModule;
import com.igknighters.subsystems.swerve.module.SwerveModuleReal;
import com.igknighters.subsystems.swerve.module.SwerveModuleSim;
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
// Field Coordinate System from the blue alliance driver station
// -------------------------
// | ^ |
// | Xpos |
// | | |
// | | |
// | <-Ypos----+----negY-> |
// | | |
// | | |
// | Xneg |
// | v |
// -------------------------
public class Swerve extends SubsystemBase {
    private final Gyro gyro;
    private final SwerveModule[] swerveMods;
    private final SwerveVisualizer visualizer;
    private final SwerveSetpointProcessor setpointProcessor = new SwerveSetpointProcessor();

    public Swerve() {

        if (RobotBase.isReal()) {
            swerveMods = new SwerveModule[] {
                    new SwerveModuleReal(ConstValues.kSwerve.Mod0.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.Mod1.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.Mod2.CONSTANTS, false),
                    new SwerveModuleReal(ConstValues.kSwerve.Mod3.CONSTANTS, false)
            };
            gyro = new GyroReal();
        } else {
            swerveMods = new SwerveModule[] {
                    new SwerveModuleSim(ConstValues.kSwerve.Mod0.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.Mod1.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.Mod2.CONSTANTS),
                    new SwerveModuleSim(ConstValues.kSwerve.Mod3.CONSTANTS)
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
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        // if (RobotBase.isReal()) speeds.omegaRadiansPerSecond *= -1.0;

        // Logger.recordOutput("Swerve/rawTargetChassisSpeed", speeds);

        // var output = setpointProcessor.processSetpoint(speeds);

        // Logger.recordOutput("Swerve/processedTargetChassisSpeed", output.chassisSpeeds());

        setModuleStates(
            kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds),
            isOpenLoop
        );
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
        var wantedAngleRads = wantedAngle.getRadians();
        var currentAngleRads = getYawRads();

        var rotVelo = kSwerve.ANGLE_CONTROLLER_KP
                * MathUtil.inputModulus(wantedAngleRads - currentAngleRads, -Math.PI, Math.PI);
        return Math.max(Math.min(rotVelo, kSwerve.MAX_ANGULAR_VELOCITY), -kSwerve.MAX_ANGULAR_VELOCITY);
    }

    public Rotation2d rotationRelativeToPose(Rotation2d wantedAngleOffet, Translation2d pose) {
        var currentTrans = getPose().getTranslation();
        var angleBetween = Math.atan2(
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
            Logger.recordOutput("Swerve/targetChassisSpeed", new ChassisSpeeds());
        }

        Logger.recordOutput("Swerve/chassisSpeed", getChassisSpeed());

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
