package com.igknighters.commands.tests;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.GeomUtil;

import edu.wpi.first.wpilibj2.command.Command;

public class Characterizers {

    public static Command characterizeSwerve(Swerve swerve) {
        return new FeedForwardCharacterization(
            swerve,
            volt -> swerve.setVoltageOut(volt, GeomUtil.ROTATION2D_ZERO),
            () -> swerve.getChassisSpeed().vxMetersPerSecond
        ).withName("Swerve Characterization");
    }
}
