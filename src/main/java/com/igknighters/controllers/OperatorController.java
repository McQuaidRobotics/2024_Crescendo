package com.igknighters.controllers;

import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;

import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.util.Channels;

import edu.wpi.first.wpilibj2.command.Commands;

public class OperatorController extends ControllerParent {
    public double frozenWristRadsOffset = 0.0;

    public OperatorController(int port) {
        super(port, true);
        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.moveTo(allss.stem.get(), StemPosition.STOW, 2.0)
                .andThen(
                    UmbrellaCommands.expell(allss.umbrella.get())
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.B.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.moveTo(allss.stem.get(), StemPosition.STOW, 2.0)
                .andThen(
                    UmbrellaCommands.expell(allss.umbrella.get())
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.X.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.moveTo(allss.stem.get(), StemPosition.STOW, 2.0)
                .andThen(
                    UmbrellaCommands.expell(allss.umbrella.get())
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.Y.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.moveTo(allss.stem.get(), StemPosition.STOW, 2.0)
                .andThen(
                    UmbrellaCommands.expell(allss.umbrella.get())
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

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
        this.DPR.binding = this.DPR.binding = this.DPL.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> {
                    Channels.Sender.broadcast("HomePivot", Boolean.class)
                            .send(true);
            }));
        }, Subsystems.Stem);

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
