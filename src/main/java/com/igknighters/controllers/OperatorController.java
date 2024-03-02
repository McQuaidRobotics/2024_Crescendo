package com.igknighters.controllers;

import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.stem.wrist.WristRealSuicidal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;

public class OperatorController extends ControllerParent {
    public double frozenWristRadsOffset = 0.0;

    public OperatorController(int port) {
        super(port, true, ControllerType.Operator);
        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        // this.A.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(
        //         Commands.sequence(
        //             Commands.runOnce(
        //                 () -> WristRealSuicidal.sweetReleaseOfDeath = false
        //             ),
        //             StemCommands.holdAt(allss.stem.get(), StemPosition.CLIMB)
        //         )
        //     );
        // }, Subsystems.Stem);

        // this.B.binding =

        // this.X.binding =

        // this.Y.binding =

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
        this.LT.binding = new Binding((trig, allss) -> {
            trig.and(this.RT.trigger).whileTrue(
                    StemCommands.LimitedManualControl(
                            allss.stem.get(),
                            this.leftStickY(),
                            this.rightStickY(),
                            0.225));
        }, Subsystems.Stem);

        // this.RT.binding = DON'T USE!!! OTHER TRIGGERS COMMANDS USES BOTH TRIGGERS!!!

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> {
        //         this.frozenWristRadsOffset -= Units.degreesToRadians(0.5);
        //         this.frozenWristRadsOffset = MathUtil.clamp(this.frozenWristRadsOffset, Units.degreesToRadians(-3.0), Units.degreesToRadians(3.0));
        //         kWrist.FROZEN_WRIST_ANGLE_WITH_OFFSET = kWrist.FROZEN_WRIST_ANGLE + frozenWristRadsOffset;
        //         WristRealSuicidal.sweetReleaseOfDeath = false;

        //     }).withName("Decrease Frozen Wrist Rads"));
        // }, Subsystems.none());

        // // this.DPL.binding =

        // this.DPU.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> {
        //         this.frozenWristRadsOffset += Units.degreesToRadians(0.5);
        //         this.frozenWristRadsOffset = MathUtil.clamp(this.frozenWristRadsOffset, Units.degreesToRadians(-3.0), Units.degreesToRadians(3.0));
        //         kWrist.FROZEN_WRIST_ANGLE_WITH_OFFSET = kWrist.FROZEN_WRIST_ANGLE + frozenWristRadsOffset;
        //         WristRealSuicidal.sweetReleaseOfDeath = false;

        //     }).withName("Increase Frozen Wrist Rads"));
        // }, Subsystems.none());
    }
}
