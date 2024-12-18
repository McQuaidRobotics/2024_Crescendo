package com.igknighters.commands.tests;

import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.GeomUtil;

import edu.wpi.first.wpilibj2.command.Command;

public class Characterizers {
    public static Command characterizePivot(Stem stem) {
        return new StaticCharacterization(
            stem,
            volt -> stem.setStemVolts(volt, 0.0, 0.0),
            () -> stem.getStemVelocities()[0]
        ).withName("Pivot Characterization");
    }

    public static Command characterizeWrist(Stem stem) {
        return new StaticCharacterization(
            stem,
            volt -> stem.setStemVolts(0.0, volt, 0.0),
            () -> stem.getStemVelocities()[1]
        ).withName("Wrist Characterization");
    }

    public static Command characterizeTelescope(Stem stem) {
        return new StaticCharacterization(
            stem,
            volt -> stem.setStemVolts(0.0, 0.0, volt),
            () -> stem.getStemVelocities()[2]
        ).withName("Telescope Characterization");
    }

    public static Command characterizeSwerve(Swerve swerve) {
        return new FeedForwardCharacterization(
            swerve,
            volt -> swerve.setVoltageOut(volt, GeomUtil.ROTATION2D_ZERO),
            () -> swerve.getChassisSpeed().vxMetersPerSecond
        ).withName("Swerve Characterization");
    }
}
