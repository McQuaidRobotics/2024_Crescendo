package com.igknighters.controllers;

import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class DriverController extends ControllerParent {

    public DriverController(int port) {
        super(port, true, ControllerType.Driver);
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
                            HigherOrderCommands.scoreAmp(
                                    allss.swerve.get(),
                                    allss.stem.get(),
                                    allss.umbrella.get()));
                },
                Subsystems.Swerve,
                Subsystems.Stem,
                Subsystems.Umbrella);

        this.X.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.holdAt(
                    allss.stem.get(),
                    StemPosition.STOW
                )
            );
        }, Subsystems.Stem);

        this.Y.binding = new Binding(
            (trig, allss) -> {
                trig.onTrue(
                    UmbrellaCommands.expell(
                        allss.umbrella.get()
                    )
                );
            },
            Subsystems.Umbrella
        );

        /// BUMPER
        // # Our main driver doesn't use bumpers
        // this.LB.binding = # Dont use

        // this.RB.binding = # Dont use

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
                        this
                    ),
                    UmbrellaCommands.spinupShooter(
                        allss.umbrella.get(),
                        kControls.SHOOTER_RPM
                    )
                ).finallyDo(
                    allss.umbrella.get()::stopAll
                ).withName("Highorder Aim")
            );
        }, Subsystems.Swerve, Subsystems.Stem, Subsystems.Umbrella);

        this.RT.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                new ProxyCommand(
                    () -> {
                        if (this.LT.trigger.getAsBoolean()) {
                            return Commands.parallel(
                                HigherOrderCommands.aim(
                                    allss.swerve.get(),
                                    allss.stem.get(),
                                    this
                                ),
                                UmbrellaCommands.shoot(
                                    allss.umbrella.get()
                                )
                            ).until(
                                () -> !this.LT.trigger.getAsBoolean()
                            ).finallyDo(
                                allss.umbrella.get()::stopAll
                            ).andThen(
                                StemCommands.holdAt(
                                    allss.stem.get(),
                                    StemPosition.STOW
                                )
                            ).withName("Highorder Aim and Shoot");
                        } else {
                            return UmbrellaCommands.shoot(
                                allss.umbrella.get()
                            ).finallyDo(
                                allss.umbrella.get()::stopAll
                            ).andThen(
                                StemCommands.holdAt(
                                    allss.stem.get(),
                                    StemPosition.STOW
                                )
                            ).withName("Shoot");
                        }
                    }
                ).withName("Proxy Shoot")
            );
        }, Subsystems.Umbrella);

        /// DPAD
        // this.DPR.binding =

        // this.DPD.binding =

        // this.DPL.binding =

        // this.DPU.binding =
    }
}
