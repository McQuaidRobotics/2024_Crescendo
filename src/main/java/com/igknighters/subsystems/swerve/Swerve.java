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

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final SwerveVisualizer visualizer;
    private final Gyro gyro;

    public Swerve() {

        if (RobotBase.isReal()) {
            swerveMods = new SwerveModule[] {
                new SwerveModuleReal(ConstValues.kSwerve.Mod0.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod1.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod2.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod3.CONSTANTS)
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
                        getPose()),
                GlobalState.LocalizerType.HYBRID);

        visualizer = new SwerveVisualizer(this, swerveMods);
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        if (RobotBase.isReal())
            speeds.omegaRadiansPerSecond *= -1.0;

        Logger.recordOutput("Swerve/targetChassisSpeed", speeds);

        setModuleStates(speeds, isOpenLoop);
    }

    /**
     * Offsets the gyro to define the current yaw as the supplied value
     * 
     * @param degrees The value to set the gyro yaw to in degrees
     */
    public void setYaw(double degrees) {
        gyro.setYawRads(Units.degreesToRadians(degrees));
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

    public void setModuleStates(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        setModuleStates(kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds), isOpenLoop);
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
