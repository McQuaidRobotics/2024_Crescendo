package com.igknighters.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPositions {
    public static Pose2d SPEAKER = new Pose2d(new Translation2d(1.28, 5.55), Rotation2d.fromDegrees(180));
    public static Pose2d AMP = new Pose2d(new Translation2d(1.82, 7.72), Rotation2d.fromDegrees(90));
    public static Pose2d STAGE_CENTER = new Pose2d(new Translation2d(5.78, 4.12), Rotation2d.fromDegrees(180));
    public static Pose2d STAGE_LEFT = new Pose2d(new Translation2d(4.39, 4.86), Rotation2d.fromDegrees(-60));
    public static Pose2d STAGE_RIGHT = new Pose2d(new Translation2d(4.38, 3.33), Rotation2d.fromDegrees(60));
    public static Pose2d NOTE_LEFT = new Pose2d(new Translation2d(2.9, 7.0), Rotation2d.fromDegrees(0.0));
    public static Pose2d NOTE_CENTER = new Pose2d(new Translation2d(2.9, 5.55), Rotation2d.fromDegrees(0.0));
    public static Pose2d NOTE_RIGHT = new Pose2d(new Translation2d(2.9, 4.10), Rotation2d.fromDegrees(0.0));
}