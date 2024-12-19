package igknighters.util;

import edu.wpi.first.math.geometry.Rotation2d;

public sealed interface ChassisSpeedsV2 {
    record FieldRelativeChassisSpeeds(double vx, double vy, double omega) implements ChassisSpeedsV2 {
        @Override
        public FieldRelativeChassisSpeeds asFieldRelative(Rotation2d robotAngle) {
            return this;
        }

        @Override
        public RobotRelativeChassisSpeeds asRobotRelative(Rotation2d robotAngle) {
            return new RobotRelativeChassisSpeeds(
                vx * robotAngle.getCos() + vy * robotAngle.getSin(),
                -vx * robotAngle.getSin() + vy * robotAngle.getCos(),
                omega
            );
        }
    }
    record RobotRelativeChassisSpeeds(double vx, double vy, double omega) implements ChassisSpeedsV2 {
        @Override
        public FieldRelativeChassisSpeeds asFieldRelative(Rotation2d robotAngle) {
            return new FieldRelativeChassisSpeeds(
                vx * robotAngle.getCos() - vy * robotAngle.getSin(),
                vx * robotAngle.getSin() + vy * robotAngle.getCos(),
                omega
            );
        }

        @Override
        public RobotRelativeChassisSpeeds asRobotRelative(Rotation2d robotAngle) {
            return this;
        }
    }

    static FieldRelativeChassisSpeeds fromFieldRelativeSpeeds(double vx, double vy, double omega) {
        return new FieldRelativeChassisSpeeds(vx, vy, omega);
    }

    static RobotRelativeChassisSpeeds fromRobotRelativeSpeeds(double vx, double vy, double omega) {
        return new RobotRelativeChassisSpeeds(vx, vy, omega);
    }

    double vx();
    double vy();
    double omega();

    FieldRelativeChassisSpeeds asFieldRelative(Rotation2d robotAngle);
    RobotRelativeChassisSpeeds asRobotRelative(Rotation2d robotAngle);
}
