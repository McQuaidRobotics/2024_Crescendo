package com.igknighters.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import org.littletonrobotics.junction.Logger;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.AngleMotorConstants;
import com.igknighters.constants.ConstValues.kSwerve.DriveMotorConstants;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.SwerveModuleConstants;

public class SwerveModuleSim implements SwerveModule {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), kSwerve.DRIVE_GEAR_RATIO, 0.025);
    private FlywheelSim angleSim = new FlywheelSim(DCMotor.getFalcon500(1), kSwerve.ANGLE_GEAR_RATIO, 0.004);

    private final PIDController driveFeedback = new PIDController(
            DriveMotorConstants.kP,
            DriveMotorConstants.kI,
            DriveMotorConstants.kD,
            ConstValues.PERIODIC_TIME);
    private final PIDController angleFeedback = new PIDController(
            AngleMotorConstants.kP,
            AngleMotorConstants.kI,
            AngleMotorConstants.kD,
            ConstValues.PERIODIC_TIME);

    public final int moduleNumber;

    private final SwerveModuleInputs inputs;

    public SwerveModuleSim(final SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleId.num;
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);

        inputs = new SwerveModuleInputs();

        inputs.angleAbsoluteRads = Math.random() * 2.0 * Math.PI;

        BootupLogger.bootupLog("    SwerveModule[" + this.moduleNumber + "] initialized (sim)");
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    private double driveRadiansToMeters(double radians) {
        return driveRotationsToMeters(radians / (2.0 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                inputs.drivePositionMeters,
                getAngle());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                inputs.driveVeloMPS,
                getAngle());
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    private Rotation2d getAngle() {
        return new Rotation2d(inputs.angleAbsoluteRads);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? new Rotation2d(inputs.angleAbsoluteRads)
                : desiredState.angle;
        inputs.targetAngleAbsoluteRads = angle.getRadians();

        var angleAppliedVolts = MathUtil.clamp(
                angleFeedback.calculate(getAngle().getRadians(), angle.getRadians()),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        angleSim.setInputVoltage(angleAppliedVolts);

        inputs.angleVolts = angleAppliedVolts;
        inputs.angleAbsoluteRads = angle.getRadians();
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        inputs.targetDriveVeloMPS = desiredState.speedMetersPerSecond;

        desiredState.speedMetersPerSecond *= Math.cos(angleFeedback.getPositionError());

        double velocityRadPerSec = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_DIAMETER / 2);
        var driveAppliedVolts = MathUtil.clamp(
                driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        driveSim.setInputVoltage(driveAppliedVolts);

        inputs.driveVolts = driveAppliedVolts;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            this.driveSim.setInputVoltage(0.0);
            this.angleSim.setInputVoltage(0.0);
        }
        driveSim.update(ConstValues.PERIODIC_TIME);
        angleSim.update(ConstValues.PERIODIC_TIME);

        inputs.drivePositionMeters += driveRadiansToMeters(
                driveSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME);

        double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;
        inputs.angleAbsoluteRads += angleDiffRad;

        while (inputs.angleAbsoluteRads < 0) {
            inputs.angleAbsoluteRads += 2 * Math.PI;
        }
        while (inputs.angleAbsoluteRads > 2 * Math.PI) {
            inputs.angleAbsoluteRads -= 2 * Math.PI;
        }

        inputs.angleVeloRadPS = angleSim.getAngularVelocityRadPerSec();
        inputs.angleAmps = angleSim.getCurrentDrawAmps();

        inputs.driveVeloMPS = driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
        inputs.driveAmps = driveSim.getCurrentDrawAmps();

        Logger.processInputs("Swerve/SwerveModule[" + this.moduleNumber + "]", inputs);
    }

    @Override
    public void setVoltageOut(double volts) {
    }
}
