package com.igknighters.commands.autos;

import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.AutoSwerveTargetSpeaker;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

public class AutosCmdRegister {
    private static void registerCommand(String name, Command command) {
        var cmd =  new WrapperCommand(command) {
            @Override
            public void initialize() {
                super.initialize();
                System.out.println("Command " + name + " started");
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                System.out.println("Command " + name + " finished");
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

        // SpecializedNamedCommands.registerCommand(
        //         "Intake",
        //         SpecializedNamedCommand.fromLambda(
        //                 (Object timeout) -> {
        //                     return HigherOrderCommands
        //                             .intakeGamepiece(stem, umbrella)
        //                             .withTimeout((Double) timeout);
        //                 },
        //                 Double.class))
        //         .withDefault(9999.0);

        // NamedCommands.registerCommand(
        //         "Stow",
        //         StemCommands.holdAt(stem, StemPosition.STOW));

        // SpecializedNamedCommands.registerCommand(
        //         "Spinup",
        //         SpecializedNamedCommand.fromLambda(
        //                 (Object rpm) -> {
        //                     return UmbrellaCommands
        //                             .spinupShooter(umbrella, (Double) rpm)
        //                                 .withName("Spinup");
        //                 },
        //                 Double.class))
        //         .withDefault(kControls.SHOOTER_RPM);

        registerCommand(
            "Intake",
            Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                UmbrellaCommands.intake(umbrella)
                .until(() -> umbrella.holdingGamepiece()))
                .andThen(StemCommands.moveTo(stem, StemPosition.SUBWOOFER))
                .withName("Intake")
        );

        registerCommand(
            "IntakeNoStow",
            Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                UmbrellaCommands.intake(umbrella)
                .until(() -> umbrella.holdingGamepiece())).withTimeout(4.0)
                .withName("IntakeNoStow")
        );

        registerCommand(
            "Spinup",
            UmbrellaCommands
                .waitUntilSpunUp(umbrella, kAuto.AUTO_SHOOTER_RPM, 0.9)
                .withName("Spinup")
        );

        registerCommand(
                "Aim",
                StemCommands.aimAtSpeaker(stem, false)
                    .withName("Aim")
        );

        registerCommand(
                "AimSub",
                StemCommands.moveTo(stem, StemPosition.SUBWOOFER)
                    .withName("AimSub")
        );

        registerCommand(
            "AutoShoot",
            Commands.parallel(
                new AutoSwerveTargetSpeaker(swerve),
                StemCommands.aimAtSpeaker(stem, true),
                UmbrellaCommands.waitUntilSpunUp(umbrella, kControls.SHOOTER_RPM, 1.2)
            ).andThen(
                UmbrellaCommands.shoot(umbrella)
            ).withName("AutoShoot")
        );

        registerCommand(
            "FeedShooter",
            UmbrellaCommands.shoot(umbrella)
                .withName("FeedShooter")
        );

        // SpecializedNamedCommands.generateSpecialized();
    }
}
