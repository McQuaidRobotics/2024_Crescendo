package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.StemPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import javax.security.auth.kerberos.KeyTab;
import com.igknighters.GlobalState;
import com.igknighters.LED;
import com.igknighters.LED.LedAnimations;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;

@SuppressWarnings("unused")

public class InspectorController extends ControllerParent {
    public InspectorController(int port) {
        super(port, true);

        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.INTAKE));
        }, Subsystems.Stem);

        this.B.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.AMP_SAFE));
        }, Subsystems.Stem);

        this.X.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.STOW));
        }, Subsystems.Stem);

        this.Y.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.STARTING));
        }, Subsystems.Stem);

        /// BUMPER
        this.LB.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.fromDegrees(30.0, 90.0, 3.0)));
        }, Subsystems.Stem);

        this.RB.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.fromDegrees(100.0, 110.0, 1.0)));
        }, Subsystems.Stem);

        /// CENTER BUTTONS
        // this.Back.binding =

        // this.Start.binding =

        /// STICKS
        // this.LS.binding =

        // this.RS.binding =

        /// TRIGGERS
        this.LT.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.fromDegrees(15.0, 105.0, 0.6581)));
        }, Subsystems.Stem);

        this.RT.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(allss.stem.get(), StemPosition.fromDegrees(40.0,
                            Units.radiansToDegrees(kWrist.MIN_ANGLE), kTelescope.MAX_METERS)));
        }, Subsystems.Stem);

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        // this.DPL.binding =

        // this.DPU.binding =
    }
}
