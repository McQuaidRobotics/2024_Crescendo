package com.igknighters.controllers;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kStem.kPivot;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

import java.util.Set;
import java.util.function.Supplier;

import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.SwerveCommands;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTargetCmd;
import com.igknighters.commands.umbrella.UmbrellaCommands;

@SuppressWarnings("unused")

/** If debug is false this controller does not initialize */
public class TestingController extends ControllerBase {
    private static final TunableDouble passDistance = TunableValues.getDouble("Test/PassTestDist", 11.0);
    private static final TunableDouble passRpm = TunableValues.getDouble("Test/PassTesRpm", 4500.0);
    public TestingController(int port, Localizer localizer) {
        super(port, Robot.isDebug());

        // disregard null safety as it is checked on assignment

        /// FACE BUTTONS
        this.A.binding = new Binding(Subsystems.Umbrella, (trig, allss) -> {
            trig.onTrue(
                    UmbrellaCommands.intake(allss.umbrella.get()));
        });

        this.B.binding = new Binding(Subsystems.Umbrella, (trig, allss) -> {
            trig.onTrue(
                    UmbrellaCommands.shoot(
                        allss.umbrella.get(),
                        () -> 1000
                    ));
        });

        this.X.binding = new Binding(Subsystems.Stem, (trig, allss) -> {
            trig.onTrue(
                    UmbrellaCommands.expell(allss.umbrella.get()));
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
        // this.LB.binding = 

        this.RB.binding = new Binding((trig, allss) -> {
            Stem stem = allss.stem.get();
            Umbrella umbrella = allss.umbrella.get();
            trig.whileTrue(
                Commands.deadline(
                    UmbrellaCommands.intakeWhileIdleShooter(umbrella, passRpm::value)
                        .until(() -> umbrella.holdingGamepiece()),
                    StemCommands.holdStill(stem)
                ).andThen(
                    new ScheduleCommand(StemCommands.holdStill(stem))
                ).withName("idk")
            );
        }, Subsystems.Stem, Subsystems.Umbrella);

        /// CENTER BUTTONS
        // this.Back.binding =

        this.Start.binding = new Binding((trig, allss) -> {
            Swerve swerve = allss.swerve.get();
            Timer timer = new Timer();
            trig.onTrue(
                Commands.defer(
                    () -> SwerveCommands.pointTowards(
                        swerve,
                        Rotation2d.fromRadians(swerve.getYawRads())
                            .plus(Rotation2d.fromDegrees(90.0))
                    ),
                    Set.of(swerve)
                ).beforeStarting(timer::restart)
                .andThen(Commands.defer(() -> Commands.print("Time: " + timer.get()), Set.of()))
                .andThen(timer::stop)
            );
        }, Subsystems.Swerve);

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

        // this.LT.binding = 

        // this.RT.binding 

        /// DPAD
        // this.DPR.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.TELEOP)).ignoringDisable(true));
        // });

        // this.DPD.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.ERROR)).ignoringDisable(true));
        // });

        // this.DPL.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.SHOOTING)).ignoringDisable(true));
        // });

        // this.DPU.binding = new Binding((trig, allss) -> {
        //     trig.onTrue(Commands.runOnce(() -> LED.sendAnimation(LedAnimations.INTAKE)).ignoringDisable(true));
        // });
    }
}
