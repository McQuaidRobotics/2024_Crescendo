package igknighters.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.ConstValues.Conv;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 */
public final class FieldConstants {

  public static final double FIELD_LENGTH = 17.548;
  public static final double FIELD_WIDTH = 8.052;
  public static final double TAPE_WIDTH = 2.0 * Conv.INCHES_TO_METERS;

  public static final AprilTagFieldLayout APRIL_TAG_FIELD =
      new AprilTagFieldLayout(
          List.of(AprilTags.TAGS), FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);

  public static final Translation2d TRANSLATION2D_CENTER =
      new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, FieldConstants.FIELD_WIDTH / 2.0);
  public static final Pose2d POSE2D_CENTER = new Pose2d(TRANSLATION2D_CENTER, Rotation2d.kZero);
}
