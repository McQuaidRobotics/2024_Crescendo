package com.igknighters.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kSwerve.kAngleMotor;
import com.igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import com.igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import com.igknighters.util.logging.BootupLogger;

public class SwerveModuleSim extends SwerveModule {
    private final FlywheelSim driveSim = new FlywheelSim(DCMotor.getFalcon500(1), kSwerve.DRIVE_GEAR_RATIO, 0.025);
    private final FlywheelSim angleSim = new FlywheelSim(DCMotor.getFalcon500(1), kSwerve.ANGLE_GEAR_RATIO, 0.004);

    private boolean gotDirectionsLastCycle = false;

    private final PIDController driveFeedback = new PIDController(
            kDriveMotor.kP,
            kDriveMotor.kI,
            kDriveMotor.kD,
            ConstValues.PERIODIC_TIME);
    private final PIDController angleFeedback = new PIDController(
            kAngleMotor.kP,
            kAngleMotor.kI,
            kAngleMotor.kD,
            ConstValues.PERIODIC_TIME);

    public final int moduleNumber;

    public SwerveModuleSim(final SwerveModuleConstants moduleConstants, SimSwerveOdometryThread odoThread) {
        this.moduleNumber = moduleConstants.getModuleId().num;

        // just to test the consts
        moduleConstants.getDriveMotorID();
        moduleConstants.getAngleMotorID();
        moduleConstants.getCancoderID();
        moduleConstants.getModuleChassisPose();

        angleFeedback.enableContinuousInput(-Math.PI, Math.PI);

        super.angleAbsoluteRads = Units.rotationsToRadians(Math.random());

        odoThread.addModulePositionSupplier(moduleNumber, this::getCurrentPosition);

        BootupLogger.bootupLog("    SwerveModule[" + this.moduleNumber + "] initialized (sim)");
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    private double driveRadiansToMeters(double radians) {
        return driveRotationsToMeters(radians / (2.0 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        gotDirectionsLastCycle = true;
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                super.drivePositionMeters,
                getAngle());
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                super.driveVeloMPS,
                getAngle());
    }

    public int getModuleNumber() {
        return this.moduleNumber;
    }

    private Rotation2d getAngle() {
        return new Rotation2d(super.angleAbsoluteRads);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? new Rotation2d(super.angleAbsoluteRads)
                : desiredState.angle;
        super.targetAngleAbsoluteRads = angle.getRadians();

        var angleAppliedVolts = MathUtil.clamp(
                angleFeedback.calculate(getAngle().getRadians(), angle.getRadians()),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        angleSim.setInputVoltage(angleAppliedVolts);

        super.angleVolts = angleAppliedVolts;
        super.angleAbsoluteRads = angle.getRadians();
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;

        desiredState.speedMetersPerSecond *= Math.cos(angleFeedback.getPositionError());

        double velocityRadPerSec = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_DIAMETER / 2);
        var driveAppliedVolts = MathUtil.clamp(
                driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadPerSec),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());
        driveSim.setInputVoltage(driveAppliedVolts);

        super.driveVolts = driveAppliedVolts;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || !gotDirectionsLastCycle) {
            this.driveSim.setInputVoltage(0.0);
            this.angleSim.setInputVoltage(0.0);
        }
        log("gotDirectionsLastCycle", gotDirectionsLastCycle);
        gotDirectionsLastCycle = false;

        driveSim.update(ConstValues.PERIODIC_TIME);
        angleSim.update(ConstValues.PERIODIC_TIME);

        super.drivePositionMeters += driveRadiansToMeters(
                driveSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME);

        double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;
        super.angleAbsoluteRads += angleDiffRad;

        while (super.angleAbsoluteRads < 0) {
            super.angleAbsoluteRads += 2 * Math.PI;
        }
        while (super.angleAbsoluteRads > 2 * Math.PI) {
            super.angleAbsoluteRads -= 2 * Math.PI;
        }

        super.angleVeloRadPS = angleSim.getAngularVelocityRadPerSec();
        super.angleAmps = angleSim.getCurrentDrawAmps();

        super.driveVeloMPS = driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
        super.driveAmps = driveSim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltageOut(double volts, Rotation2d angle) {
        super.driveVolts = volts;
        super.angleAbsoluteRads = angle.getRadians();
        super.targetAngleAbsoluteRads = angle.getRadians();
    }
}
