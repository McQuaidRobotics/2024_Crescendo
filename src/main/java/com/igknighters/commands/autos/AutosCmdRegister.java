package com.igknighters.commands.autos;

import com.igknighters.commands.HigherOrderCommands;
import com.igknighters.commands.autos.SpecializedNamedCommands.SpecializedNamedCommand;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.AutoSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.vision.Vision;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import monologue.MonoDashboard;

public class AutosCmdRegister {
    private static void logAutoEvent(String name, String event) {
        String msg = "Auto Command" + name + " " + event;
        System.out.println(msg);
        MonoDashboard.put("AutoEvent", msg);
    }

    private static void registerCommand(String name, Command command) {
        var cmd =  new WrapperCommand(command) {
            @Override
            public void initialize() {
                logAutoEvent(name, "started");
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                logAutoEvent(name, "ended");
            }
        };
        NamedCommands.registerCommand(name, cmd);
    }

    public static void registerCommands(AllSubsystems allSubsystems) {
        if (!allSubsystems.hasAllSubsystems()) {
            return;
        }

        Umbrella umbrella = allSubsystems.umbrella.get();
        Stem stem = allSubsystems.stem.get();
        Swerve swerve = allSubsystems.swerve.get();
        Vision vision = allSubsystems.vision.get();

        SpecializedNamedCommands.registerCommand(
                "Intake",
                SpecializedNamedCommand.fromLambda(
                        (Object timeout) -> {
                            return HigherOrderCommands
                                    .intakeGamepiece(stem, umbrella)
                                    .withTimeout((Double) timeout);
                        },
                        Double.class))
                .withDefault(3.0);

        SpecializedNamedCommands.registerCommand(
            "IntakeNoStow",
            SpecializedNamedCommand.fromLambda(
                (Object timeout) -> {
                    return Commands.race(
                        StemCommands.holdAt(stem, StemPosition.INTAKE),
                        UmbrellaCommands.intake(umbrella)
                            .until(() -> umbrella.holdingGamepiece())
                    ).withTimeout((Double) timeout);
                },
                Double.class)
        ).withDefault(3.0);

        registerCommand(
            "Spinup",
            Commands.run(() -> umbrella.spinupShooterToRPM(kControls.SHOOTER_RPM))
                .finallyDo(() -> umbrella.stopAll())
                .withName("Spinup")
        );

        registerCommand(
                "Expell",
                UmbrellaCommands.expell(umbrella).withTimeout(0.5)
        );

        registerCommand(
                "Aim",
                StemCommands.aimAtSpeaker(stem, false)
                    .withName("Aim")
        );

        registerCommand(
                "Stow",
                StemCommands.holdAt(
                            stem,
                            StemPosition.STOW)
        );

        registerCommand(
                "AimSub",
                StemCommands.moveTo(stem, StemPosition.STARTING)
                    .withName("AimSub")
        );

        registerCommand(
            "AutoShoot",
            Commands.parallel(
                new AutoSwerveTargetSpeaker(swerve, vision::getLatestPoseWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(stem, true)
                    .finallyDo(() -> logAutoEvent("Stem Targeting", "Done"))
            ).andThen(
                UmbrellaCommands.shootAuto(umbrella)
                    .finallyDo(() -> logAutoEvent("Shooting", "Done"))
            ).withName("AutoShoot")
        );

        registerCommand(
            "FeedShooter",
            UmbrellaCommands.shootAuto(umbrella)
                .withName("FeedShooter")
        );

        SpecializedNamedCommands.generateSpecialized();
    }
}
