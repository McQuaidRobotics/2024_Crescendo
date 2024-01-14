package com.igknighters.commands.autos;

import com.igknighters.constants.ConstValues.kAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicRoutines {
    // public static SequentialCommandGroup TRANSLATE = new SequentialCommandGroup(
    //     new DynamicPath(2.0, 1.0, 0.0).withName("path 1").getCmd(),
    //     new DynamicPath(-1.0, -2.0, 0.0).withName("path 2").getCmd()
    // );
    public static SequentialCommandGroup TRANSLATE = new SequentialCommandGroup(
        AutoBuilder.pathfindToPose(
            new Pose2d(
                new Translation2d(2.0, 1.0),
                Rotation2d.fromDegrees(0.0)
            ),
            kAuto.DYNAMIC_PATH_CONSTRAINTS,
            kAuto.DYN_END_VELO,
            0.0
        ),
        AutoBuilder.pathfindToPose(
            new Pose2d(
                new Translation2d(-1.0, -2.0),
                Rotation2d.fromDegrees(0.0)
            ),
            kAuto.DYNAMIC_PATH_CONSTRAINTS,
            kAuto.DYN_END_VELO,
            0.0
        )
    );

    public static Command[] choosableDynamicRoutines() {
        Command[] choosableRoutines = new Command[]{
            TRANSLATE.withName("TRANSLATE")
        };
        return choosableRoutines;
    }
}
