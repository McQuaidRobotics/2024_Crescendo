package igknighters.commands.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.subsystems.swerve.Swerve;

public class Characterizers {
  public static Command characterizeSwerve(Swerve swerve) {
    return new FeedForwardCharacterization(
            swerve,
            volt -> swerve.setVoltageOut(volt, Rotation2d.kZero),
            () -> swerve.getRobotSpeeds().vx())
        .withName("Swerve Characterization");
  }
}
