package com.igknighters.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise.
 */
public final class FieldConstants {
    
    public static final double FIELD_LENGTH = Units.feetToMeters(54.4);
    public static final double FIELD_WIDTH = Units.feetToMeters(26.9);
    public static final double TAPE_WIDTH = Units.inchesToMeters(2.0);
    public static final double WING_X = Units.inchesToMeters(229.201);
    public static final double PODIUM_X = Units.inchesToMeters(126.75);
    public static final double STARTING_LINE_X = Units.inchesToMeters(74.111);

    public static final Translation2d AMP_CENTER = new Translation2d(
            Units.inchesToMeters(72.455),
            Units.inchesToMeters(322.996));

    public static final class Speaker {

        /** Center of the speaker opening (blue alliance) */
        public static final Pose2d CENTER_SPEAKER_OPENING = new Pose2d(0.0, FIELD_WIDTH - Units.inchesToMeters(104.0),
                new Rotation2d());

        // corners (blue alliance origin)
        public static final Translation3d TOP_RIGHT_SPEAKER = new Translation3d(
                Units.inchesToMeters(18.055),
                Units.inchesToMeters(238.815),
                Units.inchesToMeters(13.091));

        public static final Translation3d TOP_LEFT_SPEAKER = new Translation3d(
                Units.inchesToMeters(18.055),
                Units.inchesToMeters(197.765),
                Units.inchesToMeters(83.091));

        public static final Translation3d BOTTOM_RIGHT_SPEAKER = new Translation3d(
                0.0,
                Units.inchesToMeters(238.815),
                Units.inchesToMeters(78.324));
        public static final Translation3d BOTTOM_LEFT_SPEAKER = new Translation3d(
                0.0,
                Units.inchesToMeters(197.765),
                Units.inchesToMeters(78.324));
    }

    public static final AprilTagFieldLayout APRIL_TAG_FIELD = new AprilTagFieldLayout(
            List.of(AprilTags.APRILTAGS),
            FieldConstants.FIELD_LENGTH,
            FieldConstants.FIELD_WIDTH);
}
