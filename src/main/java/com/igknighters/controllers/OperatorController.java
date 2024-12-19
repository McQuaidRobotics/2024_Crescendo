package com.igknighters.controllers;

public class OperatorController extends ControllerBase {
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

        // this.DPL.binding = 

        // this.DPU.binding =
    }
}
