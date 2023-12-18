package com.igknighters.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.AngleMotorConstants;
import com.igknighters.constants.ConstValues.kSwerve.DriveMotorConstants;
import com.igknighters.util.SwerveModuleConstants;
public class SwerveModuleSim implements SwerveModule {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0 / kSwerve.DRIVE_GEAR_RATIO, 0.025);
    private FlywheelSim angleSim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0 / kSwerve.ANGLE_GEAR_RATIO, 0.004);

    private final PIDController driveFeedback = new PIDController(DriveMotorConstants.kP, DriveMotorConstants.kI, DriveMotorConstants.kD,
            ConstValues.PERIODIC_TIME);
    private final PIDController angleFeedback = new PIDController(AngleMotorConstants.kP, AngleMotorConstants.kI, AngleMotorConstants.kD,
            ConstValues.PERIODIC_TIME);

    private double drivePositionRad = 0.0;
    private double angleAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double angleAppliedVolts = 0.0;
    private Rotation2d lastAngle = new Rotation2d();

    public int moduleNumber;

    public SwerveModuleSim(final SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleId.num;
        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                driveRotationsToMeters(drivePositionRad / (2 * Math.PI)),
                getAngle());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0),
                getAngle());
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    private Rotation2d getAngle() {
        return lastAngle;
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01)) ? lastAngle
                : desiredState.angle;

        angleAppliedVolts = MathUtil.clamp(
                angleFeedback.calculate(getAngle().getRadians(), angle.getRadians()),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        angleSim.setInputVoltage(angleAppliedVolts);

        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.speedMetersPerSecond *= Math.cos(angleFeedback.getPositionError());

        double velocityRadPerSec = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_DIAMETER / 2);
        driveAppliedVolts = MathUtil.clamp(
                driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void periodic() {
        driveSim.update(0.02);
        angleSim.update(0.02);

        drivePositionRad += driveSim.getAngularVelocityRadPerSec() * 0.02;

        double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * 0.02;
        angleAbsolutePositionRad += angleDiffRad;

        while (angleAbsolutePositionRad < 0) {
            angleAbsolutePositionRad += 2 * Math.PI;
        }
        while (angleAbsolutePositionRad > 2 * Math.PI) {
            angleAbsolutePositionRad -= 2 * Math.PI;
        }
    }
}
