package com.igknighters.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.igknighters.LED.LedPatterns;
import com.igknighters.LED;

public class LEDCommands {
    public Command commandSetLED(LedPatterns pattern) {
        return Commands.runOnce(() -> LED.getInstance().setLed(pattern));
    }

    public Command commandSetLED(LedPatterns pattern, double seconds) {
        return Commands.runOnce(() -> LED.getInstance().setLed(pattern, seconds));
    }
}
