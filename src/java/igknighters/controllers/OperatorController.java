package igknighters.controllers;

import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.commands.led.LedCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.LedAnimations;

public class OperatorController extends ControllerBase {
  public double frozenWristRadsOffset = 0.0;

  public OperatorController(int port, Subsystems subsystems) {
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

    this.Start.onTrue(LedCommands.animate(subsystems.led, LedAnimations.Test));

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
