package com.igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.igknighters.LED.LedAnimations;
import com.igknighters.LED;

public class LEDCommands {
    public Command commandSetLED(LedAnimations pattern) {
        return Commands.runOnce(() -> LED.sendAnimation(pattern))
            .withName("LED " + pattern.toString());
    }

    public Command commandSetLED(LedAnimations pattern, double seconds) {
        return Commands.runOnce(() -> LED.sendAnimation(pattern).withDuration(seconds))
            .withName("LED " + pattern.toString() + " for " + seconds + " seconds");
    }
}
