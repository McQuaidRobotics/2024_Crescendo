package com.igknighters.commands.autos;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DynamicRoutines {

    public static Command testDynPath(Swerve swerve) {
        return new SequentialCommandGroup(
            new DynamicPath(5.5, 3.0, -60.0, 0.1).withName("path 1").getCmd(swerve),
            new DynamicPath(3.55, 4.10, 180.0, 0.1).withName("path 2").getCmd(swerve),
            new DynamicPath(5.5, 5.2, 60.0, 0.1).withName("path 3").getCmd(swerve),
            new DynamicPath(5.5, 3.0, -60.0, 0.1).withName("path 4").getCmd(swerve),
            new DynamicPath(3.55, 4.10, 180, 0.1).withName("path 5").getCmd(swerve),
            new DynamicPath(5.5, 5.2, 60.0, 0.1).withName("path 6").getCmd(swerve),
            new DynamicPath(6.5, 4.1, 180.0, 0.1).withName("path 7").getCmd(swerve)
        );
    }

    public static Command[] choosableDynamicRoutines(Swerve swerve) {
        Command[] choosableRoutines = new Command[]{
            testDynPath(swerve).withName("TEST")
        };
        return choosableRoutines;
    }
}
