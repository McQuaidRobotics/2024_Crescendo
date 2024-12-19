package com.igknighters.controllers;

import com.igknighters.Localizer;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.subsystems.SubsystemResources.Subsystems;

public class DriverController extends ControllerBase {

    public DriverController(int port, Localizer localizer) {
        super(port, true);

        // disregard null safety for subsystems as it is checked on assignment

        /// FACE BUTTONS
        // this.A.binding = 

        // this.B.binding = 

        // this.X.binding = 

        // this.Y.binding = 

        /// BUMPER
        // this.RB.binding = 

        // this.LB.binding = 

        /// CENTER BUTTONS
        // this.Back.binding = 

        this.Start.binding = new Binding(Subsystems.Swerve, (trig, allss) -> {
                trig.onTrue(SwerveCommands.orientGyro(allss.swerve.get(), localizer));
        });

        /// STICKS
        // this.LS.binding = # Don't use

        // this.RS.binding = # Don't use

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
