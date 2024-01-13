package com.igknighters.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class AprilTags {
        public static final AprilTag[] APRILTAGS = new AprilTag[] {
            new AprilTag(
                1,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(593.68), Units.feetToMeters(9.68), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                2,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(637.21), Units.feetToMeters(34.79), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(120))
                )
            ),
            new AprilTag(
                3,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(652.73), Units.feetToMeters(196.17), Units.feetToMeters(57.13)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180))
                )
            ),
            new AprilTag(
                4,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(652.73), Units.feetToMeters(218.42), Units.feetToMeters(57.13)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180))
                )
            ),
            new AprilTag(
                5,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(578.77), Units.feetToMeters(323.00), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                6,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(72.5), Units.feetToMeters(323.00), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(270))
                )
            ),
            new AprilTag(
                7,
                new Pose3d(
                    new Translation3d(-Units.feetToMeters(1.50), Units.feetToMeters(218.42), Units.feetToMeters(57.13)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(0))
                )
            ),
            new AprilTag(
                8,
                new Pose3d(
                    new Translation3d(-Units.feetToMeters(1.50), Units.feetToMeters(196.17), Units.feetToMeters(57.13)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(0))
                )
            ),
            new AprilTag(
                9,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(14.02), Units.feetToMeters(34.79), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(
                10,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(57.54), Units.feetToMeters(9.68), Units.feetToMeters(53.38)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(
                11,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(468.69), Units.feetToMeters(146.19), Units.feetToMeters(52.00)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(300))
                )
            ),
            new AprilTag(
                12,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(468.69), Units.feetToMeters(177.10), Units.feetToMeters(52.00)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(60))
                )
            ),
            new AprilTag(
                13,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(441.74), Units.feetToMeters(161.62), Units.feetToMeters(52.00)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180))
                )
            ),
            new AprilTag(
                14,
                new Pose3d(
                    new Translation3d(Units.feetToMeters(209.48), Units.feetToMeters(161.62), Units.feetToMeters(52.00)),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(0))
                )
            ),
            new AprilTag(
                15,
                new Pose3d(
                    new Translation3d(182.73, 177.10, 52.00),
                    new Rotation3d(0.0, 0.0, 120)
                )
            ),
            new AprilTag(
                16,
                new Pose3d(
                    new Translation3d(182.73, 146.19, 52.00),
                    new Rotation3d(0.0, 0.0, 240)
                )
            ),
            
            
        };
    public static final AprilTagFieldLayout APRIL_TAG_FIELD = new AprilTagFieldLayout(
        List.of(APRILTAGS),
        Units.feetToMeters(54.4),
        Units.feetToMeters(26.9)
    );
}
