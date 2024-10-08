package com.igknighters.commands;

import com.igknighters.subsystems.led.Led;
import com.igknighters.subsystems.led.LedAnimations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LedCommands {
    public static Command animate(Led led, LedAnimations pattern) {
        return Commands.runOnce(() -> led.animate(pattern))
            .andThen(Commands.run(() -> {}))
            .withName("LedAnimate[" + pattern + "]");
    }

    public static Command animate(Led led, LedAnimations pattern, double timeout) {
        return Commands.runOnce(() -> led.animate(pattern))
            .andThen(Commands.waitSeconds(timeout))
            .withName("LedAnimate[" + pattern + "]");
    }
}
