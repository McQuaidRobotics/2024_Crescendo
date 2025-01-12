package igknighters.commands.swerve.teleop;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.controllers.ControllerBase;
import igknighters.subsystems.swerve.Swerve;
import wpilibExt.Speeds;

public class TeleopSwerveTraditionalCmd extends TeleopSwerveBaseCmd {

  public TeleopSwerveTraditionalCmd(Swerve swerve, ControllerBase controller) {
    super(swerve, controller);
  }

  @Override
  public void execute() {
    Translation2d vt = orientForUser(getTranslation()).times(kSwerve.MAX_DRIVE_VELOCITY);

    // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
    //         vt.getX(),
    //         vt.getY(),
    //         -getRotationX() * kSwerve.MAX_ANGULAR_VELOCITY, // invert because CCW is positive
    //         new Rotation2d(swerve.getYawRads())
    // );

    Speeds fieldSpeeds =
        Speeds.fromFieldRelative(
            vt.getX(), vt.getY(), -getRotationX() * kSwerve.MAX_ANGULAR_VELOCITY);

    swerve.drive(fieldSpeeds);
  }

  public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
