package com.igknighters.controllers;

import com.igknighters.Localizer;
import com.igknighters.Robot;

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerBase {
    public TestingController(int port, Localizer localizer) {
        super(port, Robot.isDebug());

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

        // this.RT.binding = 

        // this.LT.binding = 

        // this.RT.binding 

        /// DPAD
        // this.DPR.binding = 

        // this.DPD.binding = 

        // this.DPL.binding = 

        // this.DPU.binding = 
    }
}
