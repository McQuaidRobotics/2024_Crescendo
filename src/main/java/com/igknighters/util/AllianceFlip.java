package com.igknighters.util;

import com.igknighters.constants.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class AllianceFlip {
    /**
     * @param translation
     * @return Translation object with its x coordinate flipped over the y-axis
     */
    public static final Translation2d flipTranslation(Translation2d translation) {
        return new Translation2d(FieldConstants.FIELD_LENGTH - translation.getX(), translation.getY());
    }
    /**
     * @param translation - Translation2d or 3d
     * @return Translation object with its x coordinate flipped over the y-axis
     */
    public static final Translation3d flipTranslation(Translation3d translation) {
        return new Translation3d(FieldConstants.FIELD_LENGTH - translation.getX(), translation.getY(), translation.getZ());
    }

    /**
     * @param rotation
     * @return Rotation object flipped over the y-axis
     */
    public static final Rotation2d flipRotation(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }
    /**
     * @param rotation
     * @return Rotation object flipped over the y-axis
     */
    public static final Rotation3d flipRotation(Rotation3d rotation) {
        return new Rotation3d(
            rotation.getX(),
            rotation.getY(),
            flipRotation(Rotation2d.fromRadians(rotation.getZ())).getRadians()
        );
    }

    /**
     * @param pose
     * @return The pose object with its x coordinate flipped over the y-axis and rotated by PI or 180 degrees
     */
    public static final Pose2d flipPose(Pose2d pose) {
        return new Pose2d(
            flipTranslation(pose.getTranslation()),
            flipRotation(pose.getRotation())
        );
    }
    /**
     * @param pose
     * @return The pose object with its x coordinate flipped over the y-axis and yaw rotated by PI or 180 degrees
     */
    public static final Pose3d flipPose(Pose3d pose) {
        return new Pose3d(
            flipTranslation(pose.getTranslation()),
            flipRotation(pose.getRotation())
        );
    }

    /**
     * @param pose
     * @return The transform object with its x coordinate flipped over the y-axis and rotated by PI or 180 degrees
     */
    public static final Transform2d flipTransform(Transform2d transform) {
        return new Transform2d(
            flipTranslation(transform.getTranslation()),
            flipRotation(transform.getRotation())
        );
    }
    /**
     * @param pose
     * @return The transform object with its x coordinate flipped over the y-axis and yaw rotated by PI or 180 degrees
     */
    public static final Transform3d flipTransform(Transform3d transform) {
        return new Transform3d(
            flipTranslation(transform.getTranslation()),
            flipRotation(transform.getRotation())
        );
    }
}
