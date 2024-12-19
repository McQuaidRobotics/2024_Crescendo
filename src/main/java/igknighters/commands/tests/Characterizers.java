package igknighters.commands.tests;

import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
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
            volt -> swerve.setVoltageOut(volt, Rotation2d.kZero),
            () -> swerve.getChassisSpeed().vxMetersPerSecond
        ).withName("Swerve Characterization");
    }
}