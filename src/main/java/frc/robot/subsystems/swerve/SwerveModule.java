package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.kSwerve;
import frc.robot.util.NTpreferences;
import frc.robot.util.SwerveModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFXSimState driveSimState;
    private final StatusSignal<Double> drivePositionSignal;
    private final StatusSignal<Double> driveVelocitySignal;

    private final TalonFX angleMotor;
    private final TalonFXSimState angleSimState;
    private final StatusSignal<Double> anglePositionSignal;
    private final StatusSignal<Double> angleVelocitySignal;

    private final CANcoder angleEncoder;
    private final CANcoderSimState angleEncoderSimState;
    private final StatusSignal<Double> angleAbsoluteSignal;
    private final StatusSignal<Double> angleAbsoluteVeloSignal;

    public final int moduleNumber;
    private final Rotation2d rotationOffset;
    @SuppressWarnings("unused")
    private final Translation2d moduleChassisPose;
    private Rotation2d lastAngle = new Rotation2d();

    // SIM
    private final LinearSystemSim<N1, N1, N1> driveSim;
    private final LinearSystemSim<N2, N1, N1> rotationSim;

    public SwerveModule(final SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleId.num;
        this.rotationOffset = NTpreferences.getRotationOffset(moduleConstants.moduleId);
        this.moduleChassisPose = moduleConstants.moduleChassisPose;

        driveMotor = new TalonFX(moduleConstants.driveMotorID, kSwerve.CANBUS);
        driveSimState = driveMotor.getSimState();
        angleMotor = new TalonFX(moduleConstants.angleMotorID, kSwerve.CANBUS);
        angleSimState = angleMotor.getSimState();
        angleEncoder = new CANcoder(moduleConstants.cancoderID, kSwerve.CANBUS);
        angleEncoderSimState = angleEncoder.getSimState();

        configureDriveMotor();
        configureAngleMotor();
        configureCANcoder();

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();

        anglePositionSignal = angleMotor.getPosition();
        angleVelocitySignal = angleMotor.getVelocity();

        angleAbsoluteSignal = angleEncoder.getAbsolutePosition();
        angleAbsoluteVeloSignal = angleEncoder.getVelocity();

        driveSim = new LinearSystemSim<>(
                LinearSystemId.identifyVelocitySystem(kSwerve.Sim.DRIVE_KV, kSwerve.Sim.DRIVE_KA));
        rotationSim = new LinearSystemSim<>(
                LinearSystemId.identifyPositionSystem(kSwerve.Sim.ROTATION_KV, kSwerve.Sim.ROTATION_KA));
    }

    private void configureDriveMotor() {
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = kSwerve.DRIVE_MOTOR_INVERT;
        driveConfig.MotorOutput.NeutralMode = kSwerve.DRIVE_NEUTRAL_MODE;
        driveConfig.Slot0.kP = kSwerve.DRIVE_KP;
        driveConfig.Slot0.kI = kSwerve.DRIVE_KI;
        driveConfig.Slot0.kD = kSwerve.DRIVE_KD;
        driveConfig.Slot0.kV = 12.0 / (kSwerve.MAX_SPEED / kSwerve.METERS_PER_DRIVE_MOTOR_ROTATION);

        driveMotor.getConfigurator().apply(driveConfig);
    }

    private void configureAngleMotor() {
        var angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.Inverted = kSwerve.ANGLE_MOTOR_INVERT;
        angleConfig.MotorOutput.NeutralMode = kSwerve.ANGLE_NEUTRAL_MODE;
        angleConfig.Slot0.kP = kSwerve.ANGLE_KP;
        angleConfig.Slot0.kI = kSwerve.ANGLE_KI;
        angleConfig.Slot0.kD = kSwerve.ANGLE_KD;
        angleConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleConfig.Feedback.RotorToSensorRatio = 1.0 / kSwerve.ANGLE_MECHANISM_RATIO;
        angleConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotor.getConfigurator().apply(angleConfig);
    }

    private void configureCANcoder() {
        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.SensorDirection = kSwerve.CANCODER_INVERT;
        canCoderConfig.MagnetSensor.MagnetOffset = -rotationOffset.getRotations();

        angleEncoder.getConfigurator().apply(canCoderConfig);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveRotationsToMeters(drivePositionSignal.getValue()),
                getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public Rotation2d getAngle() {
        if (Robot.isSimulation()) {
            return lastAngle;
        } else {
            return Rotation2d.fromRotations(angleAbsoluteSignal.getValue());
        }
    }

    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle;

        SmartDashboard.putNumber("Module: " + moduleNumber + " angle", angle.getDegrees() + 180);

        angleMotor.setControl(new PositionDutyCycle(angle.getRotations()));
        lastAngle = angle;
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        SmartDashboard.putNumber("Module: " + moduleNumber + " velo", desiredState.speedMetersPerSecond);
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED;
            var controlRequest = new DutyCycleOut(percentOutput);
            driveMotor.setControl(controlRequest);
        } else {
            double rps = Math.min(desiredState.speedMetersPerSecond, kSwerve.MAX_SPEED)
                    / kSwerve.METERS_PER_DRIVE_MOTOR_ROTATION;
            var veloRequest = new VelocityVoltage(rps).withEnableFOC(true);
            driveMotor.setControl(veloRequest);
        }
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.METERS_PER_DRIVE_MOTOR_ROTATION;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveRotationsToMeters(driveVelocitySignal.getValue()),
                getAngle());
    }

    public void periodic() {
        drivePositionSignal.refresh();
        driveVelocitySignal.refresh();
        anglePositionSignal.refresh();
        angleVelocitySignal.refresh();
        angleAbsoluteSignal.refresh();
        angleAbsoluteVeloSignal.refresh();
    }

    public void simulationPeriodic() {
        driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        angleSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        angleEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        SmartDashboard.putNumber("Module: " + moduleNumber + " simvoltDrive", driveSimState.getMotorVoltage());
        SmartDashboard.putNumber("Module: " + moduleNumber + " simvoltAngle", angleSimState.getMotorVoltage());
        driveSim.setInput(driveSimState.getMotorVoltage());
        rotationSim.setInput(angleSimState.getMotorVoltage());

        driveSim.update(0.02);
        rotationSim.update(0.02);

        double driveVel = driveSim.getOutput(0) / kSwerve.METERS_PER_DRIVE_MOTOR_ROTATION;
        driveSimState.setRotorVelocity(driveVel);
        driveSimState.addRotorPosition(driveVel * 0.02);

        double rotationPos = rotationSim.getOutput(0);
        SmartDashboard.putNumber("Module: " + moduleNumber + " simrot", rotationPos);
        double rotationDeltaPos = rotationPos - anglePositionSignal.getValue();
        angleSimState.addRotorPosition(rotationDeltaPos);
        angleSimState.setRotorVelocity(rotationDeltaPos / 0.02);
        angleEncoderSimState.setRawPosition(rotationSim.getOutput(0));
        angleEncoderSimState.setVelocity(angleVelocitySignal.getValue());
    }
}
