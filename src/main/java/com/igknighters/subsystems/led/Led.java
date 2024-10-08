package com.igknighters.subsystems.led;

import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kLed;
import com.igknighters.subsystems.SubsystemResources.LockFreeSubsystem;
import com.igknighters.subsystems.led.LedAnimations.PartialAnimation;
import com.igknighters.subsystems.led.driver.CandleDriver;
import com.igknighters.subsystems.led.driver.Driver;
import com.igknighters.subsystems.led.driver.SimDriver;
import com.igknighters.util.logging.Tracer;

public class Led implements LockFreeSubsystem {
    private final Driver driver;

    public Led() {
        if (Robot.isReal()) {
            driver = new CandleDriver();
        } else {
            driver = new SimDriver();
        }
    }

    public void animate(LedAnimations animation) {
        driver.animate(
            new PartialAnimation[] {
                new PartialAnimation(kLed.LED_COUNT, kLed.CANDLE_LEDS, animation)
            }
        );
    }

    @Override
    public void periodic() {
        Tracer.startTrace("LedPeriodic");
        Tracer.traceFunc("DriverPeriodic", driver::periodic);
        Tracer.endTrace();
    }
}
