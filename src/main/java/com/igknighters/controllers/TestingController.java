package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;
import com.igknighters.util.geom.AllianceFlip;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import com.igknighters.GlobalState;
import com.igknighters.LED;
import com.igknighters.LED.LedAnimations;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;

@SuppressWarnings("unused")

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerParent {
    public TestingController(int port, boolean debug) {
        super(port, debug);

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
                                    kControls.STATIONARY_WRIST_ANGLE,
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
            trig.whileTrue(
                Commands.parallel(
                    UmbrellaCommands.spinupShooter(allss.umbrella.get(), kControls.SHOOTER_RPM, ShooterSpinupReason.None),
                    StemCommands.holdAt(allss.stem.get(), StemPosition.fromRadians(
                        Units.degreesToRadians(90.0), 
                        Units.degreesToRadians(90.0), 
                        kTelescope.MIN_METERS))
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        // this.LB.binding = 

        // this.RB.binding = 

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

        this.LT.binding = new Binding((trig, allss) -> {
            trig.whileTrue(
                    Commands.parallel(
                        StemCommands.holdAt(allss.stem.get(), StemPosition.fromRadians(
                            Units.degreesToRadians(90.0), 
                            Units.degreesToRadians(90.0), 
                            kTelescope.MIN_METERS)),
                        UmbrellaCommands.spinupShooter(
                                allss.umbrella.get(),
                                kControls.SHOOTER_RPM,
                                ShooterSpinupReason.None))
                        .finallyDo(allss.umbrella.get()::stopAll)
                        .withName("Parallel to ground"));
        }, Subsystems.Swerve, Subsystems.Stem, Subsystems.Umbrella);

        this.RT.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                new ProxyCommand(
                    () -> HigherOrderCommands.genericShoot(
                        allss.swerve.get(),
                        allss.stem.get(),
                        allss.umbrella.get(),
                        this))
            .withName("Proxy Shoot"));
        }, Subsystems.Umbrella, Subsystems.Stem, Subsystems.Swerve);

        /// DPAD
        this.DPR.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.TELEOP)).ignoringDisable(true));
        });

        this.DPD.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.ERROR)).ignoringDisable(true));
        });

        this.DPL.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.SHOOTING)).ignoringDisable(true));
        });

        this.DPU.binding = new Binding((trig, allss) -> {
            trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.INTAKE)).ignoringDisable(true));
        });
    }
}
