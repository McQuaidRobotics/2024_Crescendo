package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import com.igknighters.GlobalState;
import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.stem.StemCommands.AimStrategy;
import com.igknighters.commands.umbrella.UmbrellaCommands;

@SuppressWarnings("unused")

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerParent {
    public TestingController(int port) {
        super(port, ConstValues.DEBUG, ControllerType.Testing);

        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(
                            allss.stem.get(), StemPosition.fromDegrees(
                                    70.0,
                                    80.0,
                                    0.53)));
        });

        this.B.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    StemCommands.aimAtSpeaker(allss.stem.get(), false));
        });

        this.X.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(
                            allss.stem.get(), StemPosition.fromDegrees(
                                    11.0,
                                    72.0,
                                    kTelescope.MIN_METERS + Units.inchesToMeters(4.7))));
        });

        this.Y.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    StemCommands.moveTo(
                            allss.stem.get(), StemPosition.fromDegrees(
                                    41.0,
                                    108.0,
                                    kTelescope.MIN_METERS)));
        });

        /// BUMPER
        this.LB.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    UmbrellaCommands.spinupShooter(
                            allss.umbrella.get(),
                            3800));
        }, Subsystems.Umbrella);

        this.RB.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    UmbrellaCommands.spinupShooter(
                            allss.umbrella.get(),
                            0));
        }, Subsystems.Umbrella);

        /// CENTER BUTTONS
        // this.Back.binding =

        // this.Start.binding =

        /// STICKS
        // this.LS.binding =

        // this.RS.binding =

        /// TRIGGERS
        // this.LT.binding = new Binding((trig, allss) -> {
        // trig.whileTrue(
        // UmbrellaCommands.intake(
        // allss.umbrella.get()));
        // }, Subsystems.Umbrella);

        // this.RT.binding = new Binding((trig, allss) -> {
        // trig.onTrue(
        // UmbrellaCommands.shoot(
        // allss.umbrella.get()));
        // }, Subsystems.Umbrella);

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        // this.DPL.binding =

        // this.DPU.binding =
    }
}
