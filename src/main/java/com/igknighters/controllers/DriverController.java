package com.igknighters.controllers;

import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.commands.Emotes;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.led.LedCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetCmd;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTraditionalCmd;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.led.Led;
import com.igknighters.subsystems.led.LedAnimations;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.util.geom.AllianceFlip;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class DriverController extends ControllerBase {

    private static final Command scheduleStow(Stem stem) {
        return new ScheduleCommand(StemCommands.holdAt(stem, StemPosition.STOW));
    }

    public DriverController(int port, Localizer localizer) {
        super(port, true);

        // disregard null safety for subsystems as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding((trig, allss) -> {
            trig.onTrue(
                HigherOrderCommands.intakeGamepiece(
                    allss.stem.get(),
                    allss.umbrella.get(),
                    allss.led.get()));
        }, Subsystems.Stem, Subsystems.Umbrella, Subsystems.Led);

        this.B.binding = new Binding(
            (trig, allss) -> {
                Stem stem = allss.stem.get();
                Umbrella umbrella = allss.umbrella.get();
                trig.onTrue(
                    Commands.parallel(
                        StemCommands.holdAt(
                            stem,
                            StemPosition.AMP_SAFE),
                        UmbrellaCommands.spinupShooter(
                            umbrella,
                            kControls.SHOOTER_IDLE_RPM)
                    ).until(this.RT.trigger)
                    .andThen(
                        HigherOrderCommands.ShootSequences.ampShoot(stem, umbrella),
                        scheduleStow(stem)
                    ).withName("HighorderAmp")
                );
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
                Stem stem = allss.stem.get();
                Umbrella umbrella = allss.umbrella.get();
                trig.onTrue(
                    Commands.parallel(
                        StemCommands.holdAt(
                            stem,
                            StemPosition.STARTING),
                        UmbrellaCommands.spinupShooter(
                            umbrella,
                            Robot.isDemo() ? 2500.0 : kControls.SHOOTER_RPM)
                    ).until(this.RT.trigger)
                    .andThen(
                        HigherOrderCommands.ShootSequences.shoot(
                            stem, umbrella
                        ),
                        scheduleStow(stem)
                    ).withName("HighorderAim")
                );
            },
            Subsystems.Stem, Subsystems.Umbrella);

        /// BUMPER
        this.RB.binding = new Binding((trig, allss) -> {
            Stem stem = allss.stem.get();
            Umbrella umbrella = allss.umbrella.get();
            trig.onTrue(
                Commands.parallel(
                    StemCommands.holdAt(
                        stem,
                        StemPosition.PASS_STRAIGHT),
                    UmbrellaCommands.spinupShooter(
                        umbrella,
                        kControls.SHOOTER_PASS_RPM)
                ).until(this.RT.trigger)
                .andThen(
                    HigherOrderCommands.ShootSequences.shoot(stem, umbrella),
                    scheduleStow(stem)
                ).withName("PassStraight")
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        this.LB.binding = new Binding((trig, allss) -> {
            Swerve swerve = allss.swerve.get();
            Stem stem = allss.stem.get();
            Umbrella umbrella = allss.umbrella.get();
            Command targetPass = new TeleopSwerveTargetCmd(
                swerve,
                this,
                () -> {
                    if (AllianceFlip.isBlue()) {
                        return kControls.PASS_SHOOT_FROM_LOCATION;
                    } else {
                        return AllianceFlip.flipTranslation(kControls.PASS_SHOOT_FROM_LOCATION);
                    }
                },
                kControls.PASS_LAND_LOCATION,
                false,
                1.0
            );
            Command driveNormally = new TeleopSwerveTraditionalCmd(swerve, this);
            trig.whileTrue(
                Commands.parallel(
                    Commands.either(
                        driveNormally,
                        targetPass,
                        Robot::isDemo
                    ),
                    StemCommands.holdAt(stem, StemPosition.MANUAL_PASS),
                    UmbrellaCommands.spinupShooter(
                        umbrella,
                        kControls.SHOOTER_PASS_RPM)
                ).until(this.RT.trigger)
                .andThen(
                    HigherOrderCommands.ShootSequences.shoot(stem, umbrella),
                    scheduleStow(stem)
                ).withName("HighorderPass")
            );
        }, Subsystems.Stem, Subsystems.Umbrella, Subsystems.Swerve);

        /// CENTER BUTTONS
        this.Back.binding = new Binding(Subsystems.all(), (trig, allss) -> {
            trig.onTrue(Emotes.bopYourHead(allss));
        });

        this.Start.binding = new Binding(Subsystems.Swerve, (trig, allss) -> {
                trig.onTrue(SwerveCommands.orientGyro(allss.swerve.get(), localizer));
        });

        /// STICKS
        // this.LS.binding = # Dont use

        this.RS.binding = new Binding(Subsystems.Swerve, (trig, allss) -> {
            // cancel any command that might currently be overriding rotational control
            trig.onTrue(new TeleopSwerveTraditionalCmd(allss.swerve.get(), this));
        });

        /// TRIGGERS
        this.LT.binding = new Binding((trig, allss) -> {
            Swerve swerve = allss.swerve.get();
            Stem stem = allss.stem.get();
            Umbrella umbrella = allss.umbrella.get();
            Led led = allss.led.get();
            trig.whileTrue(
                Commands.parallel(
                    HigherOrderCommands.aim(
                        swerve, stem, this, localizer),
                    UmbrellaCommands.spinupShooter(
                        umbrella,
                        kControls.SHOOTER_RPM),
                    LedCommands.animate(led, LedAnimations.SHOOTING, 1.0)
                        .beforeStarting(Commands.waitUntil(() -> umbrella.isShooterAtSpeed(0.05)))
                ).until(this.RT.trigger)
                .andThen(
                    HigherOrderCommands.ShootSequences.autoAimShoot(
                        swerve, stem, umbrella, this, localizer),
                    scheduleStow(stem)
                ).withName("HighorderAim"));
        }, Subsystems.Swerve, Subsystems.Stem, Subsystems.Umbrella, Subsystems.Led);

        // this.RT.binding = new Binding(Subsystems.Umbrella, (trig, allss) -> {
        //     Umbrella umbrella = allss.umbrella.get();
        //     trig.onTrue(Commands.defer(
        //         () -> {
        //             if (umbrella.getDefaultCommand().isScheduled()) {
        //                 return UmbrellaCommands.shoot(umbrella, () -> kControls.SHOOTER_IDLE_RPM);
        //             } else {
        //                 return UmbrellaCommands.idleShooter(umbrella, () -> kControls.SHOOTER_IDLE_RPM)
        //                     .until(() -> true);
        //             }
        //         },
        //         Set.of(umbrella)
        //     ));
        // });

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
