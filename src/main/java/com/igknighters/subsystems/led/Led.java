package com.igknighters.subsystems.led;

import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kLed;
import com.igknighters.subsystems.SubsystemResources.LockFreeSubsystem;
import com.igknighters.subsystems.led.LedAnimations.PartialAnimation;
import com.igknighters.subsystems.led.driver.CandleDriver;
import com.igknighters.subsystems.led.driver.Driver;
import com.igknighters.subsystems.led.driver.SimDriver;
import com.igknighters.util.logging.Tracer;

import edu.wpi.first.wpilibj.DriverStation;

public class Led implements LockFreeSubsystem {
    private final Driver driver;

    private int reservedId = 0;
    private boolean reserved = false;

    public Led() {
        if (Robot.isReal()) {
            driver = new CandleDriver();
        } else {
            driver = new SimDriver();
        }
    }

    public void animate(int handle, LedAnimations animation) {
        if (handle != reservedId && handle >= 0) {
            return;
        }
        driver.animate(
            new PartialAnimation[] {
                new PartialAnimation(kLed.LED_COUNT, kLed.CANDLE_LEDS, animation)
            }
        );
    }

    public int reserve() {
        reserved = true;
        return reservedId++;
    }

    public void release(int handle) {
        if (handle == reservedId) {
            reserved = false;
        }
    }

    @Override
    public void periodic() {
        Tracer.startTrace("LedPeriodic");
        Tracer.traceFunc("DriverPeriodic", driver::periodic);

        Tracer.startTrace("DefaultPatternSetter");
        if (!reserved) {
            if (DriverStation.isAutonomousEnabled()) {
                animate(-1, LedAnimations.AUTO);
            } else if (DriverStation.isTeleopEnabled()) {
                animate(-1, LedAnimations.TELEOP);
            } else {
                animate(-1, LedAnimations.DISABLED);
            }
        }
        Tracer.endTrace();

        Tracer.endTrace();
    }
}
