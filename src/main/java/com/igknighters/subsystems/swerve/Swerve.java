package com.igknighters.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.igknighters.GlobalState;
import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final SwerveVisualizer visualizer;

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final StatusSignal<Double> gyroRollSignal;
    private final StatusSignal<Double> gyroPitchSignal;
    private final StatusSignal<Double> gyroYawSignal;

    private static class SwerveInputs implements LoggableInputs {
        public double gyroPitchRads = 0.0, gyroRollRads = 0.0, gyroYawRads = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("GyroPitchRads", gyroPitchRads);
            table.put("GyroRollRads", gyroRollRads);
            table.put("GyroYawRads", gyroYawRads);
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

        GlobalState.onceInitOdometry(
            new SwerveDrivePoseEstimator(
                kSwerve.SWERVE_KINEMATICS,
                getYawRot(),
                getModulePositions(),
                getPose())
        );

        visualizer = new SwerveVisualizer(this, swerveMods);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] mSwerveModuleStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYawRot())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.getModuleNumber()], isOpenLoop);
        }
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        if (Robot.isReal())
            speeds.omegaRadiansPerSecond *= -1;
        SwerveModuleState[] targetStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MAX_DRIVE_VELOCITY);

        SmartDashboard.putNumber("Speed X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Speed Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Speed Rot", speeds.omegaRadiansPerSecond);

        setModuleStates(targetStates);
    }

    /**
     * Sets the
     * @param val
     */
    public void setYaw(double val) {
        gyro.setYaw(val);
    }

    public Rotation2d getYawRot() {
        return Rotation2d.fromDegrees(scope0To360(this.getYaw()));
    }

    /**
     * @return The raw gyro yaw value in degrees
     */
    public Double getYaw() {
        return Units.radiansToDegrees(inputs.gyroYawRads);
    }

    /**
     * @return The raw gyro pitch value in degrees
     */
    public Double getPitch() {
        return Units.radiansToDegrees(inputs.gyroPitchRads);
    }

    /**
     * @return The raw gyro roll value in degrees
     */
    public Double getRoll() {
        return Units.radiansToDegrees(inputs.gyroRollRads);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : swerveMods) {
            modulePositions[module.getModuleNumber()] = module.getCurrentPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(desiredStates[module.getModuleNumber()], false);
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : swerveMods) {
            states[module.getModuleNumber()] = module.getCurrentState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return GlobalState.getLocalizedPose();
    }

    public void resetOdometry(Pose2d pose) {
        GlobalState.resetLocalization(getYawRot(), pose, getModulePositions());
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
                gyroPitchSignal,
                gyroRollSignal,
                gyroYawSignal
        );

        inputs.gyroPitchRads = Units.degreesToRadians(getPitch());
        inputs.gyroRollRads = Units.degreesToRadians(getRoll());
        inputs.gyroYawRads = Units.degreesToRadians(getYaw());

        for (SwerveModule module : swerveMods) {
            module.periodic();
        }

        visualizer.update(GlobalState.submitSwerveData(getYawRot(), getModulePositions()));

        Logger.processInputs("Swerve", inputs);
    }

    @Override
    public void simulationPeriodic() {

        ChassisSpeeds currentSpeeds = kSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        gyroSim.setRawYaw(
                getYawRot().getDegrees() + (Units.radiansToDegrees(currentSpeeds.omegaRadiansPerSecond) * 0.02));
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
