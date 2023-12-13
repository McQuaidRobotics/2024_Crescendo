package com.igknighters.controllers;

import com.igknighters.subsystems.Resources.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriverController extends ControllerParent {

    public DriverController(int port) {
        super(port, true, ControllerType.Driver);
        // disregard null safety for subsystems as it is checked on assignment

        /// FACE BUTTONS
        // this.A.binding = new SingleDepBinding(
        //     Subsystems.Swerve,
        //     (trigger, allSS) -> trigger.onTrue(
        //         Commands.runOnce(
        //             () -> RobotState.postControlAllocation(ControlAllocation.Manual))
        //     ));

        // this.B.binding = new SingleDepBinding(
        //     Subsystems.Swerve,
        //     (trigger, allSS) -> trigger.onTrue(
        //             new AutoDriveDynamic(allSS.swerve.get(),
        //                 new Pose2d(new Translation2d(2.1, 4.7), Rotation2d.fromDegrees(270d)))
        //     ));

        // this.X.binding = new SingleDepBinding(
        //         Subsystems.Swerve,
        //         (trigger, allSS) -> trigger.onTrue(
        //                 new AutoDriveDynamic(allSS.swerve.get(),
        //                         new Pose2d(new Translation2d(13.3, 6.6), Rotation2d.fromDegrees(90d)))));

        // this.Y.binding = new SingleDepBinding(
        //     Subsystems.Swerve,
        //     (trigger, allSS) -> trigger.onTrue(
        //         Commands.runOnce(
        //             () -> RobotState.postControlAllocation(ControlAllocation.Auto))
        //     ));

        /// BUMPER
        // this.LB.binding =

        // this.RB.binding =

        /// CENTER BUTTONS
        // this.Back.binding =

        // this.Start.binding =

        /// STICKS
        // this.LS.binding =

        // this.RS.binding =

        /// TRIGGERS
        // this.LT.binding =

        // this.RT.binding =

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        // this.DPL.binding =

        // this.DPU.binding =
    }
}
