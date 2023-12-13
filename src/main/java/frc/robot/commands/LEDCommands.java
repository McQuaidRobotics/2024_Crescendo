package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LED.LedPatterns;
import frc.robot.LED;

public class LEDCommands {
    public Command commandSetLED(LedPatterns pattern) {
        return Commands.runOnce(() -> LED.getInstance().setLed(pattern));
    }

    public Command commandSetLED(LedPatterns pattern, double seconds) {
        return Commands.runOnce(() -> LED.getInstance().setLed(pattern, seconds));
    }
}
