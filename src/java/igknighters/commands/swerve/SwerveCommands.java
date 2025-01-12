package igknighters.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Localizer;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.control.RotationalController;
import igknighters.util.AllianceFlip;
import wpilibExt.Speeds;
import wpilibExt.Speeds.RobotSpeeds;

public class SwerveCommands {
  /**
   * Gets the angle between two points
   *
   * @param currentTrans The current translation
   * @param pose The pose to get the angle to
   * @param angleOffet An offset to add to the angle
   * @return The angle between the two points
   */
  public static Rotation2d rotationRelativeToPose(Translation2d currentTrans, Translation2d pose) {
    double angleBetween =
        Math.atan2(pose.getY() - currentTrans.getY(), pose.getX() - currentTrans.getX());
    return Rotation2d.fromRadians(angleBetween);
  }

  public static Command commandStopDrives(final Swerve swerve) {
    return swerve.runOnce(() -> swerve.drive(RobotSpeeds.kZero)).withName("commandStopDrives");
  }

  public static Command orientGyro(Swerve swerve, Localizer localizer) {
    return swerve.runOnce(
        () -> {
          if (AllianceFlip.isBlue()) {
            swerve.setYaw(Rotation2d.kZero);
            var pose = new Pose2d(localizer.pose().getTranslation(), Rotation2d.kZero);
            localizer.reset(pose);
          } else {
            swerve.setYaw(Rotation2d.kPi);
            var pose = new Pose2d(localizer.pose().getTranslation(), Rotation2d.kPi);
            localizer.reset(pose);
          }
        });
  }

  private abstract static class PointTowardsCommand extends Command {
    private final Swerve swerve;
    private final RotationalController rotController;
    private double velo = 0.0;

    public PointTowardsCommand(Swerve swerve) {
      this.swerve = swerve;
      this.rotController = new RotationalController(swerve);
      addRequirements(swerve);
    }

    abstract Rotation2d getTarget();

    @Override
    public void initialize() {
      rotController.reset();
    }

    @Override
    public void execute() {
      velo = rotController.calculate(getTarget().getRadians(), 0.0);
      swerve.drive(Speeds.fromFieldRelative(0.0, 0.0, velo));
    }

    @Override
    public boolean isFinished() {
      return Math.abs(velo) < 0.05;
    }

    @Override
    public String getName() {
      return "PointTowards";
    }
  }

  public static Command pointTowards(Swerve swerve, Translation2d target) {
    return new PointTowardsCommand(swerve) {
      @Override
      Rotation2d getTarget() {
        return rotationRelativeToPose(Translation2d.kZero, target);
      }
    };
  }

  public static Command pointTowards(Swerve swerve, Rotation2d target) {
    return new PointTowardsCommand(swerve) {
      @Override
      Rotation2d getTarget() {
        return target;
      }
    };
  }

  public static Command driveChassisSpeed(Swerve swerve, final Speeds speeds) {
    return Commands.run(() -> swerve.drive(speeds), swerve);
  }
}
