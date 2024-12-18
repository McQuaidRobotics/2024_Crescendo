package com.igknighters.commands.led;

import java.util.concurrent.atomic.AtomicInteger;

import com.igknighters.subsystems.led.Led;
import com.igknighters.subsystems.led.LedAnimations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedCommands {
    public static Command animate(Led led, LedAnimations pattern) {
        return animate(led, pattern, 9999.0);
    }

    public static Command animate(Led led, LedAnimations pattern, double timeout) {
        AtomicInteger handle = new AtomicInteger();
        return Commands.runOnce(() -> handle.set(led.reserve()))
            .andThen(() -> led.animate(handle.get(), pattern))
            .finallyDo(() -> led.release(handle.get()))
            .withTimeout(timeout)
            .withName("LedAnimate[" + pattern + "]");
    }
}
