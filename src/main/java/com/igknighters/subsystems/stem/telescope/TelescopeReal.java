package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.HardwareIndex.StemHW;
import com.igknighters.util.FaultManager;
import com.igknighters.util.SafeTalonFXConfiguration;


public class TelescopeReal implements Telescope {
    private final TalonFX motor;
    private final Pigeon2 gyro;

    private final StatusSignal <Double> motorVolts, motorTemp, motorAmps, motorVelo, motorRots;
    private final StatusSignal<Double> gyroMeasurement;
    private final StatusSignal<ForwardLimitValue> forwardLimitSwitch;
    private final StatusSignal<ReverseLimitValue> reverseLimitSwitch;

    private final TelescopeInputs inputs;

    public TelescopeReal(){
        motor = new TalonFX(kTelescope.MOTOR_ID);
        motor.getConfigurator().apply(motorConfig());

        motorRots = motor.getRotorPosition();
        motorVelo = motor.getRotorVelocity();
        motorAmps = motor.getTorqueCurrent();
        motorVolts = motor.getMotorVoltage();
        motorTemp = motor.getDeviceTemp();

        motorRots.setUpdateFrequency(100);
        motorVelo.setUpdateFrequency(100);
        motorAmps.setUpdateFrequency(100);
        motorVolts.setUpdateFrequency(100);
        motorTemp.setUpdateFrequency(4);
        
        forwardLimitSwitch = motor.getForwardLimit();
        reverseLimitSwitch = motor.getReverseLimit();

        motor.optimizeBusUtilization();

        gyro = new Pigeon2(kTelescope.PIGEON_ID);
        gyroMeasurement = gyro.getPitch();

        gyroMeasurement.setUpdateFrequency(100);

        gyro.optimizeBusUtilization();

        inputs = new TelescopeInputs(kTelescope.MIN_METERS);
    }
    
    private TalonFXConfiguration motorConfig() {
        TalonFXConfiguration telescopeMotorCfg = new SafeTalonFXConfiguration();
        telescopeMotorCfg.Slot0.kP = kTelescope.MOTOR_kP;
        telescopeMotorCfg.Slot0.kI = kTelescope.MOTOR_kI;
        telescopeMotorCfg.Slot0.kD = kTelescope.MOTOR_kD;
        //TODO add gravity 

        telescopeMotorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        telescopeMotorCfg.MotorOutput.Inverted = kTelescope.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        
        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitEnable = false;
        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitEnable = false;

        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
        telescopeMotorCfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = mechMetersToMotorRots(0.2);

        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        telescopeMotorCfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = mechMetersToMotorRots(0.2);


        return telescopeMotorCfg;
    }



    private double motorRotsToMechMeters(Double motorRots){
        return 0; //ctre closed loop docs
    }

    private double mechMetersToMotorRots(Double mechMeters){
        return 0; //ctre closed loop docs
    }


    @Override
    public void setVoltageOut(double volts) {
        inputs.targetMeters = 0.0;
        this.motor.setVoltage(volts);
    }

    @Override
    public void setTelescopeMeters(double meters) {
        inputs.targetMeters = meters;
        var posControlRequest = new MotionMagicVoltage(mechMetersToMotorRots(meters));//check to add with limit forward/reverse motion
        this.motor.setControl(posControlRequest);
    }

    @Override
    public double getTelescopeMeters() {
        return inputs.meters;
    }

    @Override
    public void stopMechanism(){
        this.motor.setVoltage(0);
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        return inputs.isLimitFwdSwitchHit;
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        return inputs.isLimitRevSwitchHit;
    }


    @Override
    public void periodic(){
        FaultManager.captureFault(
            StemHW.TelescopeMotor,
            BaseStatusSignal.refreshAll(
                motorRots, motorVelo,
                motorVolts, motorAmps,
                motorTemp));
        FaultManager.captureFault(
            StemHW.Pigeon2, //TODO add another pigeon to Stem Hardware
            gyroMeasurement.refresh().getStatus());
    
        inputs.meters = motorRotsToMechMeters(motorRots.getValue());
        inputs.metersPerSecond = motorRotsToMechMeters(motorVelo.getValue());
        inputs.volts = motorVolts.getValue();
        inputs.temp = motorTemp.getValue();
        inputs.amps =  motorAmps.getValue();
        inputs.isLimitFwdSwitchHit = forwardLimitSwitch.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.isLimitRevSwitchHit = reverseLimitSwitch.getValue() == ReverseLimitValue.ClosedToGround;



        Logger.processInputs("Stem/Telescope", inputs);

    }

}
