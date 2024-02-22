package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;

@SuppressWarnings("unused")

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerParent {
    public TestingController(int port) {
        super(port, ConstValues.DEBUG, ControllerType.Testing);

        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new SingleDepBinding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(
                            allss.stem.get(), StemPosition.fromDegrees(
                                    70.0,
                                    80.0,
                                    0.53)));
        });

        this.B.binding = new SingleDepBinding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                new ProxyCommand(() -> {
                    return StemCommands.moveTo(allss.stem.get(), StemPosition.fromRadians(
                        kPivot.PIVOT_MIN_RADIANS + (Math.random() * (kPivot.PIVOT_MAX_RADIANS - kPivot.PIVOT_MIN_RADIANS)), 
                        kWrist.MIN_ANGLE + (Math.random() * (kWrist.MAX_ANGLE - kWrist.MIN_ANGLE)), 
                        kTelescope.MIN_METERS + (Math.random() * (kTelescope.MAX_METERS - kTelescope.MIN_METERS))));
                })
            );
        });

        // this.X.binding = 

        this.Y.binding = new SingleDepBinding(Subsystems.Umbrella, (trig, allss) -> {
            trig.onTrue(UmbrellaCommands.waitUntilSpunUp(allss.umbrella.get(), 5000));
        });

        /// BUMPER
        this.LB.binding = new SingleDepBinding(Subsystems.Umbrella, (trig, allss) -> {
            trig.onTrue(UmbrellaCommands.intake(allss.umbrella.get()));
        });

        this.RB.binding = new SingleDepBinding(Subsystems.Umbrella, (trig, allss) -> {
            trig.onTrue(UmbrellaCommands.shoot(allss.umbrella.get()));
        });

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
