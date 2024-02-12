package com.igknighters.constants;

import java.util.List;

import com.igknighters.constants.ConstValues.Conv;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise.
 */
public final class FieldConstants {

    public static final double FIELD_LENGTH = 54.4 * Conv.FEET_TO_METERS;
    public static final double FIELD_WIDTH = 26.9 * Conv.FEET_TO_METERS;
    public static final double TAPE_WIDTH = 2.0 * Conv.INCHES_TO_METERS;
    public static final double WING_X = 229.201 * Conv.INCHES_TO_METERS;
    public static final double PODIUM_X = 126.75 * Conv.INCHES_TO_METERS;
    public static final double STARTING_LINE_X = 74.111 * Conv.INCHES_TO_METERS;

    public static final Translation2d AMP_CENTER = new Translation2d(
        1.84,
        8.2042
    );

    public static final class Speaker {

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

        public static final Translation3d SPEAKER_CENTER = TOP_LEFT_SPEAKER
                .interpolate(TOP_LEFT_SPEAKER, 0.5)
                .interpolate(
                        BOTTOM_LEFT_SPEAKER.interpolate(BOTTOM_RIGHT_SPEAKER, 0.5),
                        0.5);
    }

    public static final AprilTagFieldLayout APRIL_TAG_FIELD = new AprilTagFieldLayout(
            List.of(AprilTags.APRILTAGS),
            FieldConstants.FIELD_LENGTH,
            FieldConstants.FIELD_WIDTH);
}
