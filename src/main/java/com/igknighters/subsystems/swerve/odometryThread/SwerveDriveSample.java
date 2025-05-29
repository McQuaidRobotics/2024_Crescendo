package com.igknighters.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record SwerveDriveSample(
    SwerveModulePosition[] modulePositions,
    Rotation2d gyroYaw,
    double gforce,
    double timestamp
) {}
