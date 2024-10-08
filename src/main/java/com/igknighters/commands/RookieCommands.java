package com.igknighters.commands;

import com.igknighters.subsystems.rookie.Acquisition;

import edu.wpi.first.wpilibj2.command.Command;

public class RookieCommands {

    public static Command homeAcquisition(Acquisition acquisition) {
        return acquisition.run(() -> acquisition.setArmVoltageOut(-5.0))
            .until(acquisition::isArmCurrentTripped)
            .withTimeout(2.0)
            .andThen(() -> acquisition.setArmVoltageOut(0.0))
            .andThen(acquisition::homeArmHere)
            .withName("HomeAcquisition");
    }

    public static Command stowAcquisition(Acquisition acquisition) {
        //TODO: get stow pos
        return acquisition.run(() -> acquisition.setArmPosition(0));
    }

    public static Command intakeAcquisition(Acquisition acquisition) {
        return acquisition.run(() -> {
                acquisition.setRollerVoltageOut(-12.0);
                //TODO: get intake pos
                acquisition.setArmPosition(0.0);
            })
            .until(acquisition::isRollerCurrentTripped)
            .andThen(() -> acquisition.setRollerVoltageOut(0.0))
            .andThen(stowAcquisition(acquisition))
            .withName("IntakeAcquisition");
    }
}
