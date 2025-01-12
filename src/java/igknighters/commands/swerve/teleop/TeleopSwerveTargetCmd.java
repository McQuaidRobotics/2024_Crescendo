package igknighters.commands.swerve.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import igknighters.Localizer;
import igknighters.commands.swerve.SwerveCommands;
import igknighters.constants.ConstValues.kControls;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.controllers.ControllerBase;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.swerve.control.RotationalController;
import igknighters.util.AllianceFlip;
import java.util.function.Supplier;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;

public class TeleopSwerveTargetCmd extends TeleopSwerveBaseCmd {

  private static final Rotation2d offset = Rotation2d.kPi.plus(Rotation2d.fromDegrees(1.0));

  private final Supplier<Translation2d> translationSupplier;
  private final Translation2d target;
  private final boolean movementComp;
  private final RotationalController rotController;
  private final double speedMult;

  public TeleopSwerveTargetCmd(
      Swerve swerve,
      ControllerBase controller,
      Localizer localizer,
      Translation2d target,
      boolean movementComp,
      double speedScalar) {
    this(swerve, controller, localizer::translation, target, movementComp, speedScalar);
  }

  public TeleopSwerveTargetCmd(
      Swerve swerve,
      ControllerBase controller,
      Supplier<Translation2d> poseSupplier,
      Translation2d target,
      boolean movementComp,
      double speedScalar) {
    super(swerve, controller);
    addRequirements(swerve);
    this.translationSupplier = poseSupplier;
    this.target = target;
    this.movementComp = movementComp;
    this.rotController = new RotationalController(swerve);
    this.speedMult = speedScalar;
  }

  @Override
  public void initialize() {
    rotController.reset();
  }

  @Override
  public void execute() {
    Translation2d currentTranslation = translationSupplier.get();
    Translation2d targetTranslation =
        AllianceFlip.isBlue() ? target : AllianceFlip.flipTranslation(target);

    Translation2d vt =
        orientForUser(getTranslation()).times(kSwerve.MAX_DRIVE_VELOCITY * speedMult);

    FieldSpeeds currentChassisSpeeds = swerve.getFieldSpeeds();

    double heuristicVX = (vt.getX() + currentChassisSpeeds.vx()) / 2.0;
    double heuristicVY = (vt.getY() + currentChassisSpeeds.vy()) / 2.0;

    Translation2d lookaheadTranslation =
        currentTranslation.plus(
            new Translation2d(
                heuristicVX * kControls.SOTM_LOOKAHEAD_TIME,
                heuristicVY * kControls.SOTM_LOOKAHEAD_TIME));

    Rotation2d targetAngle;

    if (movementComp) {
      targetAngle =
          SwerveCommands.rotationRelativeToPose(lookaheadTranslation, targetTranslation)
              .plus(offset);
    } else {
      targetAngle =
          SwerveCommands.rotationRelativeToPose(currentTranslation, targetTranslation).plus(offset);
    }

    double omega = rotController.calculate(targetAngle.getRadians(), Units.degreesToRadians(0.3));

    swerve.drive(Speeds.fromFieldRelative(vt.getX(), vt.getY(), omega));
  }

  public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
