package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.subsystems.Subsystems;

public class DriverController extends ControllerBase {

  public DriverController(int port, Localizer localizer, Subsystems subsystems) {
    super(port, true);

    // disregard null safety for subsystems as it is checked on assignment

    /// FACE BUTTONS
    this.A.onTrue(Commands.none());

    this.B.onTrue(Commands.none());

    this.X.onTrue(Commands.none());

    this.Y.onTrue(Commands.none());

    /// BUMPER
    this.RB.onTrue(Commands.none());

    this.LB.onTrue(Commands.none());

    /// CENTER BUTTONS
    this.Back.onTrue(Commands.none());

    this.Start.onTrue(SwerveCommands.orientGyro(subsystems.swerve, localizer));

    /// STICKS
    this.LS.onTrue(Commands.none());

    this.RS.onTrue(Commands.none());

    /// TRIGGERS
    this.LT.onTrue(Commands.none());

    this.RT.onTrue(Commands.none());

    /// DPAD
    this.DPR.onTrue(Commands.none());

    this.DPD.onTrue(Commands.none());

    this.DPL.onTrue(Commands.none());

    this.DPU.onTrue(Commands.none());
  }
}
