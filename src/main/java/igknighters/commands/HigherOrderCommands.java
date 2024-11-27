package igknighters.commands;

import igknighters.Localizer;
import igknighters.commands.led.LedCommands;
import igknighters.commands.stem.StemCommands;
import igknighters.commands.swerve.teleop.TeleopSwerveTargetCmd;
import igknighters.commands.umbrella.UmbrellaCommands;
import igknighters.constants.FieldConstants;
import igknighters.constants.ConstValues.kControls;
import igknighters.constants.ConstValues.kStem.kTelescope;
import igknighters.constants.ConstValues.kStem.kWrist;
import igknighters.controllers.ControllerBase;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedAnimations;
import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.stem.StemPosition;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.umbrella.Umbrella;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class HigherOrderCommands {

    public static Command intakeGamepiece(Stem stem, Umbrella umbrella, Led led) {
        return Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                Commands.idle().until(
                    () -> {
                        var pose = stem.getStemPosition();
                        return pose.wristRads > (StemPosition.INTAKE.wristRads
                                + kWrist.MIN_ANGLE) / 2.0
                                && pose.telescopeMeters > kTelescope.MIN_METERS;
                }).andThen(
                    UmbrellaCommands.intake(umbrella)
                        .until(() -> umbrella.holdingGamepiece()))
                )
            .andThen(
                new ScheduleCommand(
                    StemCommands.holdAt(stem, StemPosition.STOW),
                    LedCommands.animate(led, LedAnimations.Intake, 1.0)
                )
            ).withName("Intake");
    }

    public static Command aim(
            Swerve swerve,
            Stem stem,
            ControllerBase controller,
            Localizer localizer
    ) {
        return Commands.parallel(
                new TeleopSwerveTargetCmd(
                    swerve,
                    controller,
                    localizer,
                    FieldConstants.SPEAKER.toTranslation2d(),
                    true,
                    0.25
                ),
                StemCommands.aimAtSpeaker(stem, false, localizer::pose, swerve::getChassisSpeed)).withName("Aim");
    }

    public static Command aimNotePass(
        Swerve swerve,
        Stem stem,
        ControllerBase controller,
        Localizer localizer
    ) {
        return Commands.parallel(
            new TeleopSwerveTargetCmd(
                swerve,
                controller,
                localizer,
                kControls.PASS_LAND_LOCATION,
                true,
                0.4
            ),
            StemCommands.aimAtPassPoint(
                stem,
                kControls.PASS_LAND_LOCATION,
                false,
                localizer::pose
            )
        );
    }

    public static class ShootSequences {
        private static double targetRpm(Umbrella umbrella) {
            return (umbrella.getShooterTargetSpeed() / 60.0) * (2.0 * Math.PI);
        }
        public static Command shoot(
                Stem stem,
                Umbrella umbrella
        ) {
            return Commands.deadline(
                UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella)),
                StemCommands.holdStill(stem)
            ).withName("Shoot");
        }

        public static Command autoAimShoot(
                Swerve swerve,
                Stem stem,
                Umbrella umbrella,
                ControllerBase controller,
                Localizer localizer
        ) {
            return Commands.parallel(
                HigherOrderCommands.aim(
                    swerve,
                    stem,
                    controller,
                    localizer),
                UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella))
            ).until(() -> controller.leftTrigger(true).getAsDouble() < 0.5)
            .withName("AutoAimShoot");
        }

        public static Command autoAimPassShoot(
                Swerve swerve,
                Stem stem,
                Umbrella umbrella,
                ControllerBase controller,
                Localizer localizer
        ) {
            return Commands.parallel(
                HigherOrderCommands.aimNotePass(
                    swerve,
                    stem,
                    controller,
                    localizer),
                    UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella))
            ).withName("AutoAimShoot");
        }

        public static Command ampShoot(
                Stem stem,
                Umbrella umbrella
        ) {
            return Commands.sequence(
                StemCommands.moveTo(stem, StemPosition.AMP_SCORE, 1.4),
                umbrella.run(
                    () -> {
                        umbrella.spinupShooter(kControls.SHOOTER_RPM);
                        umbrella.runIntakeAt(-1.0, true);
                    }).withTimeout(0.3),
                StemCommands.moveTo(stem, StemPosition.AMP_SAFE, 1.5)
            ).withName("AmpShoot");
        }
    }
}
