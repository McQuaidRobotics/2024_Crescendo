package frc.robot.subsystems.super_structure.pivot;

import javax.net.ssl.KeyStoreBuilderParameters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.kSuperStructure;
import frc.robot.Constants.kSuperStructure.kPivot;
import frc.robot.util.ShuffleboardApi.ShuffleEntryContainer;

public class PivotReal implements Pivot {

    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;
    private final Pigeon2 gyro;

    private final StatusSignal<Double> motorRots, motorVelo, motorAmps, motorVolts;
    private final StatusSignal<Double> gyroPitch;

    private final LinearFilter ampWindow = LinearFilter.movingAverage(25);
    private Double ampWindowVal = 0.0;

    private Boolean isHomed = false;
    private Double cachedPivotDegrees;

    private Double mechDegreesToMotorRots(Double mechanismDegrees) {
        return (mechanismDegrees / 360.0) / kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechDegrees(Double motorRots) {
        return motorRots * 360.0 * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    public PivotReal() {
        gyro = new Pigeon2(kPivot.PIGEON_ID, kSuperStructure.CANBUS);
        gyroPitch = gyro.getPitch();

        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID, kSuperStructure.CANBUS);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID, kSuperStructure.CANBUS);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        followerMotor.setControl(
                new Follower(kPivot.LEFT_MOTOR_ID, true));

        leaderMotor.setRotorPosition(mechDegreesToMotorRots(getPivotDegreesPigeon()));
        cachedPivotDegrees = getPivotDegreesPigeon();

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        motorAmps = leaderMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();
    }

    private TalonFXConfiguration getMotorConfig() {
        TalonFXConfiguration motorCfg = new TalonFXConfiguration();
        motorCfg.Slot0.kP = kPivot.MOTOR_kP;
        motorCfg.Slot0.kI = kPivot.MOTOR_kI;
        motorCfg.Slot0.kD = kPivot.MOTOR_kD;

        motorCfg.MotionMagic.MotionMagicCruiseVelocity = kPivot.MAX_VELOCITY;
        motorCfg.MotionMagic.MotionMagicAcceleration = kPivot.MAX_ACCELERATION;
        motorCfg.MotionMagic.MotionMagicJerk = kPivot.MAX_JERK;

        motorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorCfg.MotorOutput.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP;
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public Boolean setPivotDegrees(Double degrees) {
        isHomed = false;
        var posControlRequest = new MotionMagicDutyCycle(mechDegreesToMotorRots(degrees));
        this.leaderMotor.setControl(posControlRequest);
        return Math.abs(degrees - getPivotDegrees()) < kPivot.TOLERANCE;
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        isHomed = false;
        var percentControlRequest = new DutyCycleOut(percentOut);
        this.leaderMotor.setControl(percentControlRequest);
    }

    @Override
    public void stopMechanism() {
        this.leaderMotor.setVoltage(0.0);
    }

    @Override
    public Double getPivotDegrees() {
        return cachedPivotDegrees;
    }

    private Double getPivotDegreesPigeon() {
        return gyroPitch.refresh().getValue() - 1.85;
    }

    private void seedPivot() {
        var pigeonDegrees = mechDegreesToMotorRots(getPivotDegreesPigeon());
        leaderMotor.setRotorPosition(pigeonDegrees);
        cachedPivotDegrees = pigeonDegrees;
    }

    @Override
    public Boolean homeMechanism(boolean force) {
        if (force) {
            isHomed = false;
        }
        if (isHomed) {
            this.stopMechanism();
            return true;
        }

        var reached = this.setPivotDegrees(kPivot.HOME_DEGREES);
        if (reached) {
            this.stopMechanism();
            seedPivot();
            isHomed = true;
            return isHomed;
        }
        if (motorAmps.getValue() > kPivot.CURRENT_PEAK_FOR_HOME) {
            this.stopMechanism();
            seedPivot();
            isHomed = true;
        }

        return isHomed;
    }

    @Override
    public Double getRecentCurrent() {
        return ampWindowVal;
    }

    @Override
    public void brake(Boolean toBrake) {
        var motorOutputCfg = new MotorOutputConfigs();
        motorOutputCfg.NeutralMode = toBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorOutputCfg.Inverted = kPivot.INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        leaderMotor.getConfigurator().apply(motorOutputCfg);
        if (toBrake) {
            leaderMotor.setControl(new StaticBrake());
        } else {
            leaderMotor.setControl(new CoastOut());
        }
    }

    @Override
    public void setupShuffleboard(ShuffleEntryContainer tab) {
        tab.addDouble("Pivot Motor Rots", motorRots::getValue);
        tab.addDouble("Pivot Motor Velo", motorVelo::getValue);
        tab.addDouble("Pivot Motor Amps", motorAmps::getValue);
        tab.addDouble("Pivot Motor Volts", motorVolts::getValue);
        tab.addDouble("Pivot Degrees Gyro", this::getPivotDegreesPigeon);
        tab.addBoolean("Pivot Homed", () -> isHomed);
    }

    @Override
    public void periodic() {
        motorRots.refresh(); motorVelo.refresh();
        motorAmps.refresh(); motorVolts.refresh();

        cachedPivotDegrees = motorRotsToMechDegrees(motorRots.getValue());
        ampWindowVal = ampWindow.calculate(motorAmps.getValue());
    }
}
