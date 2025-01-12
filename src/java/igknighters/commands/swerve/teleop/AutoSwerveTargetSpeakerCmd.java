package igknighters.commands.swerve.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.control.RotationalController;
import igknighters.util.AllianceFlip;
import java.util.function.Supplier;
import wpilibExt.Speeds;
import wpilibExt.Speeds.RobotSpeeds;

public class AutoSwerveTargetSpeakerCmd extends Command {

  private final Swerve swerve;
  private final Supplier<Pose2d> poseSupplier;
  private final RotationalController rotController;
  private boolean isDone = false;

  public AutoSwerveTargetSpeakerCmd(Swerve swerve, Supplier<Pose2d> poseSupplier) {
    addRequirements(swerve);
    this.poseSupplier = poseSupplier;
    this.swerve = swerve;
    this.rotController = new RotationalController(swerve);
  }

  @Override
  public void initialize() {
    rotController.reset();
    isDone = false;
  }

  @Override
  public void execute() {
    Translation2d speaker = Translation2d.kZero;
    Translation2d targetTranslation =
        AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

    Rotation2d targetAngle =
        SwerveCommands.rotationRelativeToPose(
                poseSupplier.get().getTranslation(), targetTranslation)
            .plus(Rotation2d.kPi);
    double rotVelo = rotController.calculate(targetAngle.getRadians(), Units.degreesToRadians(1.5));

    if (Math.abs(rotVelo) < 0.01
        && Math.abs(
                MathUtil.angleModulus(targetAngle.getRadians())
                    - MathUtil.angleModulus(swerve.getYawRads()))
            < Units.degreesToRadians(1.5)) {
      isDone = true;
    }

    swerve.drive(Speeds.fromRobotRelative(0.0, 0.0, rotVelo));
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(RobotSpeeds.kZero);
  }
}
