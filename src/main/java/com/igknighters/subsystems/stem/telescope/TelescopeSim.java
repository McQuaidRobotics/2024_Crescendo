package com.igknighters.subsystems.stem.telescope;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.util.BootupLogger;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;;

public class TelescopeSim implements Telescope {
    private final TelescopeInputs inputs;
    private final ElevatorSim sim;
    private final PIDController pidController;
    private final SimBoolean fwdLimitSwitch, revLimitSwitch;

    public TelescopeSim() {
        pidController = new PIDController(
                kTelescope.MOTOR_kP,
                kTelescope.MOTOR_kI,
                kTelescope.MOTOR_kD);

        sim = new ElevatorSim(
                DCMotor.getFalcon500(1),
                kTelescope.MOTOR_TO_MECHANISM_RATIO,
                10.0,
                0.0,
                kTelescope.MIN_METERS,
                kTelescope.MAX_METERS,
                false,
                kTelescope.MIN_METERS);

        inputs = new TelescopeInputs(kTelescope.MIN_METERS);

        if (RobotBase.isReal()) {
            fwdLimitSwitch = null;
            revLimitSwitch = null;
            NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("TelescopeFwdLimitSwitch")
                    .getEntry("tripped")
                    .setBoolean(false);
            NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("TelescopeRevLimitSwitch")
                    .getEntry("tripped")
                    .setBoolean(false);
        } else if (GlobalState.isUnitTest()) {
            fwdLimitSwitch = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
            revLimitSwitch = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
        } else {
            fwdLimitSwitch = SimDevice.create("TelescopeFwdLimitSwitch").createBoolean("tripped", Direction.kInput,
                    false);
            revLimitSwitch = SimDevice.create("TelescopeRevLimitSwitch").createBoolean("tripped", Direction.kInput,
                    false);
        }

        BootupLogger.bootupLog("    Telescope initialized (sim)");
    }

    @Override
    public void setVoltageOut(double volts) {
        sim.setInputVoltage(volts);
        inputs.volts = volts;
        inputs.targetMeters = 0;
    }

    @Override
    public void setTelescopeMeters(double meters) {
        inputs.targetMeters = meters; // set target meters to what we want
        double telescopeVoltageFeedback = pidController.calculate(inputs.meters, meters); // makes a voltage feedback
                                                                                          // with the new controller
                                                                                          // output
        sim.setInputVoltage(telescopeVoltageFeedback); // sets voltage to the feedback
        inputs.volts = telescopeVoltageFeedback;
    }

    private void setSimStateMeters(double meters) {
        inputs.volts = 0.0;
        inputs.amps = 0.0;
        sim.setState(meters, 0.0);
    }

    @Override
    public double getTelescopeMeters() {
        return inputs.meters;
    }

    @Override
    public boolean isFwdLimitSwitchHit() {
        return fwdLimitSwitch.get();
    }

    @Override
    public boolean isRevLimitSwitchHit() {
        return revLimitSwitch.get();
    }

    @Override
    public boolean hasHomed() {
        return true;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            sim.setInputVoltage(0);
        }

        sim.update(ConstValues.PERIODIC_TIME);

        inputs.meters = sim.getPositionMeters();
        inputs.metersPerSecond = sim.getVelocityMetersPerSecond();
        inputs.temp = 0.0;
        inputs.amps = sim.getCurrentDrawAmps();

        if (RobotBase.isReal()) {
            inputs.isLimitFwdSwitchHit = NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("TelescopeFwdLimitSwitch")
                    .getEntry("tripped")
                    .setBoolean(false);
            inputs.isLimitRevSwitchHit = NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("TelescopeRevLimitSwitch")
                    .getEntry("tripped")
                    .setBoolean(false);
        } else {
            inputs.isLimitFwdSwitchHit = fwdLimitSwitch.get();
            inputs.isLimitRevSwitchHit = revLimitSwitch.get();
        }

        if (isFwdLimitSwitchHit())
            setSimStateMeters(kTelescope.MAX_METERS);

        if (isRevLimitSwitchHit())
            setSimStateMeters(kTelescope.MIN_METERS);

        Logger.processInputs("Stem/Telescope", inputs);
    }

}
