package com.igknighters.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

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
import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.module.SwerveModule;
import com.igknighters.subsystems.swerve.module.SwerveModuleReal;
import com.igknighters.subsystems.swerve.module.SwerveModuleSim;
import com.igknighters.util.Tracer;
import com.igknighters.constants.ConstValues;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final SwerveVisualizer visualizer;

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final StatusSignal<Double> gyroRollSignal;
    private final StatusSignal<Double> gyroPitchSignal;
    private final StatusSignal<Double> gyroYawSignal;
    private double simYawOffset = 0.0;

    private static class SwerveInputs implements LoggableInputs {
        public double gyroPitchRads = 0.0, gyroRollRads = 0.0, gyroYawRads = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("GyroPitchRads", gyroPitchRads);
            table.put("GyroRollRads", gyroRollRads);
            table.put("GyroYawRads", gyroYawRads);

            if (ConstValues.DEBUG) {
                table.put("#Human/GyroPitchDegrees", Units.radiansToDegrees(gyroPitchRads));
                table.put("#Human/GyroRollDegrees", Units.radiansToDegrees(gyroRollRads));
                table.put("#Human/GyroYawDegrees", Units.radiansToDegrees(gyroYawRads));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            gyroPitchRads = table.get("GyroPitchRads", gyroPitchRads);
            gyroRollRads = table.get("GyroRollRads", gyroRollRads);
            gyroYawRads = table.get("GyroYawRads", gyroYawRads);
        }
    }

    private SwerveInputs inputs = new SwerveInputs();

    public Swerve() {

        gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyroSim = gyro.getSimState();
        setYaw(0.0);
        gyroRollSignal = gyro.getRoll();
        gyroPitchSignal = gyro.getPitch();
        gyroYawSignal = gyro.getYaw();

        gyroRollSignal.setUpdateFrequency(100);
        gyroPitchSignal.setUpdateFrequency(100);
        gyroYawSignal.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();

        swerveMods = Robot.isReal() ? new SwerveModule[] {
                new SwerveModuleReal(ConstValues.kSwerve.Mod0.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod1.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod2.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod3.CONSTANTS)
        }
                : new SwerveModule[] {
                        new SwerveModuleSim(ConstValues.kSwerve.Mod0.CONSTANTS),
                        new SwerveModuleSim(ConstValues.kSwerve.Mod1.CONSTANTS),
                        new SwerveModuleSim(ConstValues.kSwerve.Mod2.CONSTANTS),
                        new SwerveModuleSim(ConstValues.kSwerve.Mod3.CONSTANTS)
                };

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
        if (Robot.isReal()) {
            gyro.setYaw(degrees);
        } else {
            simYawOffset = degrees - getYawWrappedRot().getDegrees();
        }
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
        return inputs.gyroYawRads;
    }

    /**
     * @return The raw gyro pitch value in radians
     */
    public double getPitchRads() {
        return inputs.gyroPitchRads;
    }

    /**
     * @return The raw gyro roll value in radians
     */
    public double getRollRads() {
        return inputs.gyroRollRads;
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

        BaseStatusSignal.refreshAll(
                gyroPitchSignal,
                gyroRollSignal,
                gyroYawSignal);

        inputs.gyroPitchRads = Units.degreesToRadians(gyroPitchSignal.getValue());
        inputs.gyroRollRads = Units.degreesToRadians(gyroRollSignal.getValue());
        inputs.gyroYawRads = Units.degreesToRadians(gyroYawSignal.getValue());

        for (SwerveModule module : swerveMods) {
            Tracer.traceFunc("SwerveModule[" + module.getModuleNumber() + "]", module::periodic);
        }

        visualizer.update(GlobalState.submitSwerveData(getYawWrappedRot(), getModulePositions()));

        Logger.processInputs("Swerve", inputs);

        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Swerve/targetChassisSpeed", new ChassisSpeeds());
        }

        Logger.recordOutput("Swerve/chassisSpeed", getChassisSpeed());

        Tracer.endTrace();
    }

    @Override
    public void simulationPeriodic() {

        gyroSim.setRawYaw(
                getYawWrappedRot().getDegrees()
                        + (Units.radiansToDegrees(getChassisSpeed().omegaRadiansPerSecond) * ConstValues.PERIODIC_TIME)
                        + simYawOffset);
        simYawOffset = 0.0;
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
