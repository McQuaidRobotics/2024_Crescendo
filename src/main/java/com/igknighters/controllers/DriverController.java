package com.igknighters.controllers;

import java.util.Set;

import com.igknighters.LED;
import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.LED.LedAnimations;
import com.igknighters.commands.Emotes;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

public class DriverController extends ControllerParent {

    public DriverController(int port, Localizer localizer) {
        super(port, true);

        // disregard null safety for subsystems as it is checked on assignment

        /// FACE BUTTONS
        // this.A.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(
        //         HigherOrderCommands.intakeGamepiece(
        //             allss.stem.get(),
        //             allss.umbrella.get()));
        // }, Subsystems.Stem, Subsystems.Umbrella);

        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                HigherOrderCommands.aimNotePass(
                    allss.stem.get(),
                    allss.umbrella.get(),
                    localizer
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.B.binding = new Binding(
            (trig, allss) -> {
                trig.onTrue(
                    Commands.parallel(
                        StemCommands.holdAt(
                            allss.stem.get(),
                            StemPosition.AMP_SAFE),
                        UmbrellaCommands.spinupShooter(
                            allss.umbrella.get(),
                            kControls.SHOOTER_IDLE_RPM,
                            ShooterSpinupReason.Amp))
                        .finallyDo(() -> allss.umbrella.get().stopAll())
                        .withName("AimAmp"));
            },
            Subsystems.Stem,
            Subsystems.Umbrella);

        this.X.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.holdAt(
                    allss.stem.get(),
                    StemPosition.STOW));
        }, Subsystems.Stem);

        this.Y.binding = new Binding(
            (trig, allss) -> {
                trig.onTrue(
                    Commands.parallel(
                        StemCommands.holdAt(
                            allss.stem.get(),
                            StemPosition.STARTING),
                        UmbrellaCommands.spinupShooter(
                            allss.umbrella.get(),
                            kControls.SHOOTER_RPM,
                            ShooterSpinupReason.ManualAimSpeaker))
                        .finallyDo(
                            () -> allss.umbrella.get().stopAll()
                        ).withName("SubwooferAim"));
            },
            Subsystems.Stem, Subsystems.Umbrella);

        /// BUMPER
        // # Our main driver doesn't use bumpers
        if (Robot.isDemo()) {
            this.LB.binding = new Binding(
                (trig, allss) -> {
                    trig.onTrue(
                        Commands.parallel(
                            StemCommands.holdAt(
                                allss.stem.get(),
                                StemPosition.DEMO_LOW),
                            UmbrellaCommands.spinupShooter(
                                allss.umbrella.get(),
                                3000.0,
                                ShooterSpinupReason.ManualAimSpeaker))
                            .finallyDo(
                                () -> allss.umbrella.get().stopAll()
                            ).withName("DemoLow"));
                },
                Subsystems.Stem, Subsystems.Umbrella);

                this.RB.binding = new Binding(
                    (trig, allss) -> {
                        trig.onTrue(
                            Commands.parallel(
                                StemCommands.holdAt(
                                    allss.stem.get(),
                                    StemPosition.DEMO_HIGH),
                                UmbrellaCommands.spinupShooter(
                                    allss.umbrella.get(),
                                    3000.0,
                                    ShooterSpinupReason.ManualAimSpeaker))
                                .finallyDo(
                                    () -> allss.umbrella.get().stopAll()
                                ).withName("DemoHigh"));
                    },
                    Subsystems.Stem, Subsystems.Umbrella);
        }

        /// CENTER BUTTONS
        this.Back.binding = new Binding(Subsystems.all(), (trig, allss) -> {
            trig.onTrue(Emotes.bopYourHead(allss));
        });

        this.Start.binding = new Binding(Subsystems.Swerve, (trig, allss) -> {
                trig.onTrue(SwerveCommands.orientGyro(allss.swerve.get(), localizer));
        });

        /// STICKS
        // # Our main driver doesn't use sticks
        // this.LS.binding = # Dont use

        // this.RS.binding = # Dont use

        /// TRIGGERS
        this.LT.binding = new Binding((trig, allss) -> {
            Swerve swerve = allss.swerve.get();
            Stem stem = allss.stem.get();
            Umbrella umbrella = allss.umbrella.get();
            trig.whileTrue(
                Commands.parallel(
                    HigherOrderCommands.aim(
                        swerve, stem, this, localizer),
                    UmbrellaCommands.spinupShooter(
                        umbrella,
                        kControls.SHOOTER_RPM,
                        ShooterSpinupReason.AutoAimSpeaker),
                    Commands.run(
                        () -> {
                            if (umbrella.isShooterAtSpeed(0.05)) {
                                LED.sendAnimation(LedAnimations.SHOOTING).withDuration(1.0);
                            }
                        }
                    ))
                    .finallyDo(allss.umbrella.get()::stopAll)
                    .withName("HighorderAim"));
        }, Subsystems.Swerve, Subsystems.Stem, Subsystems.Umbrella);

        this.RT.binding = new Binding((trig, allss) -> {
            var swerve = allss.swerve.get();
            var stem = allss.stem.get();
            var umbrella = allss.umbrella.get();
                trig.onTrue(
                    new DeferredCommand(
                        () -> HigherOrderCommands.genericShoot(
                            swerve,
                            stem,
                            umbrella,
                            this,
                            localizer
                        ),
                        Set.of(swerve, stem, umbrella)
                    ).withName("DeferedShoot")
                );
        }, Subsystems.Umbrella, Subsystems.Stem, Subsystems.Swerve);

        /// DPAD
        this.DPR.binding = this.DPR.binding = this.DPL.binding = new Binding((trig, allss) -> {
            Stem stem = allss.stem.get();
            trig.onTrue(stem.runOnce(() -> {
                    stem.home();
            }).withName("HomePivot"));
        }, Subsystems.Stem);

        this.DPD.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                StemCommands.moveTo(allss.stem.get(), StemPosition.STOW, 2.0)
                .andThen(
                    UmbrellaCommands.expell(allss.umbrella.get())
                )
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.DPL.binding = new Binding((trig, allss) -> {
                trig.onTrue(Commands.runOnce(() -> {
                    allss.stem.get().stopMechanisms();
                    allss.umbrella.get().stopAll();
                }).withName("StopAll"));
        }, Subsystems.Stem, Subsystems.Umbrella); 

        if (Robot.isDemo()) {
            this.DPU.binding = new Binding((trig, allss) -> {
                trig.onTrue(Emotes.yes(allss));
            }, Subsystems.Stem, Subsystems.Umbrella);
        } else {
            this.DPU.binding = new Binding((trig, allss) -> {
                trig.onTrue(StemCommands.holdAt(allss.stem.get(), StemPosition.STARTING));
            }, Subsystems.Stem);
        }
    }
}
