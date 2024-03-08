package com.igknighters.controllers;

import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class DriverController extends ControllerParent {

    public DriverController(int port) {
        super(port, true);
        // disregard null safety for subsystems as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    HigherOrderCommands.intakeGamepiece(
                            allss.stem.get(),
                            allss.umbrella.get()));
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.B.binding = new Binding(
                (trig, allss) -> {
                    trig.onTrue(
                            Commands.parallel(
                                    StemCommands.holdAt(
                                            allss.stem.get(),
                                            StemPosition.AMP),
                                    UmbrellaCommands.spinupShooter(
                                            allss.umbrella.get(),
                                            1000,
                                            ShooterSpinupReason.Amp))
                                    .finallyDo(
                                            () -> allss.umbrella.get().stopAll()));
                },
                Subsystems.Stem,
                Subsystems.Umbrella);

        this.X.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                    StemCommands.holdAt(
                            allss.stem.get(),
                            StemPosition.FROZEN_WRIST_STOW));
        }, Subsystems.Stem);

        this.Y.binding = new Binding(
                (trig, allss) -> {
                    trig.onTrue(
                            Commands.parallel(
                                    StemCommands.holdAt(
                                            allss.stem.get(),
                                            StemPosition.SUBWOOFER),
                                    UmbrellaCommands.spinupShooter(
                                            allss.umbrella.get(),
                                            kControls.SHOOTER_RPM,
                                            ShooterSpinupReason.ManualAimSpeaker))
                                    .finallyDo(
                                            () -> {
                                                allss.umbrella.get().stopAll();
                                            }));
                },
                Subsystems.Stem,
                Subsystems.Umbrella);

        /// BUMPER
        // # Our main driver doesn't use bumpers
        this.LB.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.or(RB.trigger).onTrue(
                    StemCommands.holdAt(
                            allss.stem.get(),
                            StemPosition.SUBWOOFER));
        });

        // this.RB.binding = # Is used as an or with LB

        /// CENTER BUTTONS
        // this.Back.binding =

        this.Start.binding = new Binding(Subsystems.Swerve, (trig, allss) -> {
            trig.onTrue(SwerveCommands.orientGyro(allss.swerve.get()));
        });

        /// STICKS
        // # Our main driver doesn't use sticks
        // this.LS.binding = # Dont use

        // this.RS.binding = # Dont use

        /// TRIGGERS
        this.LT.binding = new Binding((trig, allss) -> {
            trig.whileTrue(
                    Commands.parallel(
                            HigherOrderCommands.aim(
                                    allss.swerve.get(),
                                    allss.stem.get(),
                                    this),
                            UmbrellaCommands.spinupShooter(
                                    allss.umbrella.get(),
                                    kControls.AUTO_AIM_SHOOTER_RPM,
                                    ShooterSpinupReason.AutoAimSpeaker))
                            .finallyDo(
                                    allss.umbrella.get()::stopAll)
                            .withName("Highorder Aim"));
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
        // this.DPR.binding =

        // this.DPD.binding =

        this.DPL.binding = new Binding((trig, allss) -> {
                trig.onTrue(Commands.runOnce(() -> {
                        allss.stem.get().stopMechanisms();
                        allss.umbrella.get().stopAll();
                }));
        }, Subsystems.Stem, Subsystems.Umbrella); 

        this.DPU.binding = new Binding((trig, allss) -> {
            trig.onTrue(StemCommands.holdAt(allss.stem.get(), StemPosition.STARTING));
        }, Subsystems.Stem);
    }
}
