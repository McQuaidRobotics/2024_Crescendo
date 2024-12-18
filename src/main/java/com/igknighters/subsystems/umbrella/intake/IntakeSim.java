package com.igknighters.subsystems.umbrella.intake;

import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeSim extends Intake {


    public IntakeSim() {
        BootupLogger.bootupLog("    Intake initialized (sim)");
    }

    public void setExitBeam(boolean broken) {
        super.exitBeamBroken = broken;
    }

    @Override
    public void setVoltageOut(double volts) {
        setVoltageOut(volts, false);
    }

    @Override
    public boolean isExitBeamBroken() {
        return super.exitBeamBroken;
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (isExitBeamBroken() && !force) {
            volts = 0.0;
        }

        if ((force && volts < 0.1) || volts > 0.1) {
            setExitBeam(false);
        }

        super.voltsLower = volts;
        super.voltsUpper = super.voltsLower * kIntake.UPPER_DIFF;
        super.radiansPerSecondLower = (volts / RobotController.getBatteryVoltage()) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
        super.radiansPerSecondUpper = super.radiansPerSecondLower * kIntake.UPPER_DIFF;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) setVoltageOut(0.0);
    }
}
