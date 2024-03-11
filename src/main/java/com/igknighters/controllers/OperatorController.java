package com.igknighters.controllers;

import com.igknighters.subsystems.SubsystemResources.Subsystems;

import edu.wpi.first.wpilibj2.command.Commands;

public class OperatorController extends ControllerParent {
    public double frozenWristRadsOffset = 0.0;

    public OperatorController(int port) {
        super(port, true);
        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        // this.A.binding = 

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
        // this.LT.binding = 

        // this.RT.binding = DON'T USE!!! OTHER TRIGGERS COMMANDS USES BOTH TRIGGERS!!!

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        this.DPL.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> {
                    allss.stem.get().stopMechanisms();
                    allss.umbrella.get().stopAll();
            }));
    }, Subsystems.Stem, Subsystems.Umbrella);

        // this.DPU.binding = 
    }
}
