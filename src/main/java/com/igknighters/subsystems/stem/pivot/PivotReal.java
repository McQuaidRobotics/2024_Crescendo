package com.igknighters.subsystems.stem.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import com.igknighters.constants.ConstValues.kStem.kPivot;

public class PivotReal implements Pivot {
    /** Left */
    private final TalonFX leaderMotor;
    /** Right */
    private final TalonFX followerMotor;

    private final Pigeon2 gyro;

    private final StatusSignal<Double> motorRots, motorVelo, motorVolts;
    private final StatusSignal<Double> leftMotorAmps, rightMotorAmps, leftMotorTemp, rightMotorTemp;
    private final StatusSignal<Double> gyroPitch;

    private final PivotInputs inputs;


    private Double mechRadiansToMotorRots(Double mechRads) {
        return (mechRads / (2 * Math.PI)) / kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    private Double motorRotsToMechRadians(Double motorRots) {
        return motorRots * (2 * Math.PI) * kPivot.MOTOR_TO_MECHANISM_RATIO;
    }

    public PivotReal() {
        gyro = new Pigeon2(kPivot.PIGEON_ID); //add canbus
        gyroPitch = (gyro.getPitch());

        gyroPitch.setUpdateFrequency(100);
        
        gyro.optimizeBusUtilization();

        leaderMotor = new TalonFX(kPivot.LEFT_MOTOR_ID);
        followerMotor = new TalonFX(kPivot.RIGHT_MOTOR_ID);
        leaderMotor.getConfigurator().apply(getMotorConfig());
        followerMotor.getConfigurator().apply(getMotorConfig());

        followerMotor.setControl(
                new Follower(kPivot.LEFT_MOTOR_ID, true));

        inputs = new PivotInputs(gyroPitch.getValue() - kPivot.PIGEON_OFFSET);
        leaderMotor.setPosition(mechRadiansToMotorRots(getPivotRadiansPigeon()));

        motorRots = leaderMotor.getRotorPosition();
        motorVelo = leaderMotor.getRotorVelocity();
        rightMotorAmps = leaderMotor.getStatorCurrent();
        leftMotorAmps = followerMotor.getStatorCurrent();
        motorVolts = leaderMotor.getSupplyVoltage();
        rightMotorTemp = leaderMotor.getDeviceTemp();
        leftMotorTemp = followerMotor.getDeviceTemp();

        motorRots.setUpdateFrequency(100);
        motorVelo.setUpdateFrequency(100);
        rightMotorAmps.setUpdateFrequency(100);
        leftMotorAmps.setUpdateFrequency(100);
        motorVolts.setUpdateFrequency(100);
        rightMotorTemp.setUpdateFrequency(4);
        leftMotorTemp.setUpdateFrequency(4);

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();

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

        motorCfg.HardwareLimitSwitch.ReverseLimitEnable = true;
        motorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        motorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0.0;

        motorCfg.Voltage.PeakForwardVoltage = kPivot.VOLTAGE_COMP; //11.8
        motorCfg.Voltage.PeakReverseVoltage = -kPivot.VOLTAGE_COMP;

        return motorCfg;
    }

    @Override
    public void setPivotRadians(double radians) {
        var posControlRequest = new MotionMagicDutyCycle(Units.radiansToRotations(radians));
        this.leaderMotor.setControl(posControlRequest);
    }

    @Override
    public void setVoltageOut(double volts) {
        inputs.targetRadians = 0.0;
        this.leaderMotor.setVoltage(volts);
    }


    @Override
    public double getPivotRadians() {
        return inputs.radians;
    }

    private Double getPivotRadiansPigeon() {
        return inputs.gyroPitchRadians - kPivot.PIGEON_OFFSET;
    }

    private void seedPivot() {
        var pigeonRadians = mechRadiansToMotorRots(getPivotRadiansPigeon());
        leaderMotor.setPosition(pigeonRadians);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            motorRots, motorVelo,
            motorVolts, gyroPitch,
            leftMotorAmps, rightMotorAmps,
            leftMotorTemp, rightMotorTemp
        );

        inputs.radians = motorRotsToMechRadians(motorRots.getValue());
        inputs.radiansPerSecond = motorRotsToMechRadians(motorVelo.getValue());
        inputs.gyroPitchRadians = Units.degreesToRadians(gyroPitch.getValue());
        inputs.volts = motorVolts.getValue();
        inputs.leftAmps = leftMotorAmps.getValue();
        inputs.rightAmps = rightMotorAmps.getValue();
        inputs.leftTemp = leftMotorTemp.getValue();
        inputs.rightTemp = rightMotorTemp.getValue();

        Logger.processInputs("SuperStructure/Pivot", inputs);
    }


}
