package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.wpilibj2.command.Commands;

import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.commands.umbrella.UmbrellaCommands;

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerParent {
    public TestingController(int port) {
        super(port, ConstValues.DEBUG, ControllerType.Testing);

        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> {
                allss.stem.get().setStemPosition(StemPosition.fromDegrees(80.0, 0.0, 0.0));
            }));
        }, Subsystems.Stem);

        this.B.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> {
                allss.stem.get().setStemPosition(StemPosition.fromDegrees(20.0, 0.0, 0.0));
            }));
        }, Subsystems.Stem);

        // this.X.binding =

        // this.Y.binding =

        /// BUMPER
        // this.LB.binding =

        // this.RB.binding =

        /// CENTER BUTTONS
        // this.Back.binding =

        this.Start.binding = new Binding((trig, allss) -> {
            trig.whileTrue(
                UmbrellaCommands.spinupShooter(
                    allss.umbrella.get(),
                    3800
                )
            );
        }, Subsystems.Umbrella);

        /// STICKS
        // this.LS.binding =

        // this.RS.binding =

        /// TRIGGERS
        this.LT.binding = new Binding((trig, allss) -> {
            trig.whileTrue(
                UmbrellaCommands.intake(
                    allss.umbrella.get()
                )
            );
        }, Subsystems.Umbrella);

        this.RT.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                UmbrellaCommands.shoot(
                    allss.umbrella.get()
                )
            );
        }, Subsystems.Umbrella);

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        // this.DPL.binding =

        // this.DPU.binding =
    }
}
