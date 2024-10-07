package com.igknighters.subsystems.swerve.odometryThread;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;

public record SwerveDriveSample(
    SwerveDriveWheelPositions modulePositions,
    Rotation2d gyroYaw,
    double gforce,
    double timestamp
) {}
