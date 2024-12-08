package igknighters.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import sham.ShamSwerveModule;
import sham.ShamMechanism.MechanismOutputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kSwerve.kSteerMotor;
import igknighters.constants.ConstValues.kSwerve.kDriveMotor;
import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;

public class SwerveModuleSim2 extends SwerveModule {

    private boolean gotDirectionsLastCycle = false;

    private final PIDController driveFeedback;
    private final PIDController steerFeedback;

    public final int moduleId;

    private MechanismOutputs driveState = MechanismOutputs.zero();
    private MechanismOutputs steerState = MechanismOutputs.zero();

    private Voltage driveAppliedVoltage = Volts.zero();
    private Voltage steerAppliedVoltage = Volts.zero();

    public SwerveModuleSim2(final int moduleId, SimSwerveOdometryThread odoThread, ShamSwerveModule sim) {
        super("SwerveModule[" + moduleId + "]");
        this.moduleId = moduleId;

        odoThread.addModulePositionSupplier(moduleId, this::getCurrentPosition);

        driveFeedback = new PIDController(
            kDriveMotor.kP,
            kDriveMotor.kI,
            kDriveMotor.kD,
            ConstValues.PERIODIC_TIME
        );

        steerFeedback = new PIDController(
            kSteerMotor.kP,
            kSteerMotor.kI,
            kSteerMotor.kD,
            ConstValues.PERIODIC_TIME
        );
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

        BootupLogger.bootupLog("    SwerveModule[" + this.moduleId + "] initialized (sim)");
    }

    private double driveRotationsToMeters(double rotations) {
        return rotations * kSwerve.WHEEL_CIRCUMFERENCE;
    }

    private double driveRadiansToMeters(double radians) {
        return driveRotationsToMeters(radians / (2.0 * Math.PI));
    }

    public void setDesiredState(AdvancedSwerveModuleState desiredState) {
        gotDirectionsLastCycle = true;
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                super.drivePositionMeters,
                Rotation2d.fromRadians(steerState.position().in(Radians))
        );
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                super.driveVeloMPS,
                Rotation2d.fromRadians(steerState.position().in(Radians))
        );
    }

    @Override
    public int getModuleId() {
        return this.moduleId;
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_DRIVE_VELOCITY * 0.01))
                ? new Rotation2d(super.steerAbsoluteRads)
                : desiredState.angle;
        super.targetSteerAbsoluteRads = angle.getRadians();

        var feedback = MathUtil.clamp(
                steerFeedback.calculate(steerState.position().in(Radians), angle.getRadians()),
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        steerAppliedVoltage = Volts.of(feedback);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        super.targetDriveVeloMPS = desiredState.speedMetersPerSecond;

        desiredState.speedMetersPerSecond *= Math.cos(steerFeedback.getError());

        double velocityRadPerSec = desiredState.speedMetersPerSecond / (kSwerve.WHEEL_DIAMETER / 2);
        var feedback = MathUtil.clamp(
                driveFeedback.calculate(driveState.velocity().in(RadiansPerSecond), velocityRadPerSec),
                -1.0 * RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage());

        driveAppliedVoltage = Volts.of(feedback);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || !gotDirectionsLastCycle) {
            driveAppliedVoltage = Volts.zero();
            steerAppliedVoltage = Volts.zero();
        }
        log("gotDirectionsLastCycle", gotDirectionsLastCycle);
        gotDirectionsLastCycle = false;

        // super.drivePositionMeters += driveRadiansToMeters(
        //         driveSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME);

        // double angleDiffRad = angleSim.getAngularVelocityRadPerSec() * ConstValues.PERIODIC_TIME;
        // super.angleAbsoluteRads += angleDiffRad;

        // while (super.angleAbsoluteRads < 0) {
        //     super.angleAbsoluteRads += 2 * Math.PI;
        // }
        // while (super.angleAbsoluteRads > 2 * Math.PI) {
        //     super.angleAbsoluteRads -= 2 * Math.PI;
        // }

        // super.angleVeloRadPS = angleSim.getAngularVelocityRadPerSec();
        // super.angleAmps = angleSim.getCurrentDrawAmps();

        // super.driveVeloMPS = driveRotationsToMeters(driveSim.getAngularVelocityRPM() / 60.0);
        // super.driveAmps = driveSim.getCurrentDrawAmps();

        super.driveVolts = driveAppliedVoltage.in(Volts);
        super.steerVolts = steerAppliedVoltage.in(Volts);
    }

    @Override
    public void setVoltageOut(double volts, Rotation2d angle) {
        driveAppliedVoltage = Volts.of(volts);
        super.steerAbsoluteRads = angle.getRadians();
        super.targetSteerAbsoluteRads = angle.getRadians();
    }
}
