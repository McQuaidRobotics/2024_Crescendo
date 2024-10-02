package com.igknighters.commands.autos;

import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.commands.stem.StemCommands;
import com.igknighters.commands.swerve.teleop.AutoSwerveTargetSpeakerCmd;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.stem.StemSolvers.AimSolveStrategy;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;
import com.igknighters.subsystems.vision.Vision;

import choreo.ChoreoAutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import monologue.Monologue;

public class AutoCommands {
    protected final Swerve swerve;
    protected final Stem stem;
    protected final Umbrella umbrella;
    protected final Vision vision;
    protected final Localizer localizer;

    protected AutoCommands(Swerve swerve, Stem stem, Umbrella umbrella, Vision vision, Localizer localizer) {
        this.swerve = swerve;
        this.stem = stem;
        this.umbrella = umbrella;
        this.vision = vision;
        this.localizer = localizer;
    }

    protected void logAutoEvent(String name, String event) {
        String msg = "Auto Command " + name + " " + event;
        if (Robot.isDebug()) System.out.println(msg);
        Monologue.log("AutoEvent", msg);
    }

    protected Command loggedCmd(Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                logAutoEvent(this.getName(), "started");
                super.initialize();
            }

            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                logAutoEvent(this.getName(), "ended");
            }
        };
    }


    protected Command intakeGamepieceNoStow() {
        return loggedCmd(
            Commands.race(
                StemCommands.holdAt(stem, StemPosition.INTAKE),
                UmbrellaCommands.intakeWWhileIdleShooter(umbrella, UmbrellaCommands::defaultIdleRPM)
                    .until(() -> umbrella.holdingGamepiece())
            ).withName("IntakeGamepieceNoStow")
        );
    }

    protected Command aimStem(ChoreoAutoTrajectory traj) {
        return loggedCmd(
            StemCommands.aimAtSpeaker(
                stem,
                AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                false,
                traj::getFinalPose,
                swerve::getChassisSpeed
            ).withName("AimStem")
        );
    }

    protected Command aimVision() {
        return loggedCmd(
            StemCommands.aimAtSpeaker(
                stem,
                AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                false,
                vision::getLatestPoseWithFallback,
                swerve::getChassisSpeed
            ).withName("AimVision")
        );
    }

    protected Command stow() {
        return loggedCmd(
            StemCommands.moveTo(
                stem,
                StemPosition.STOW
            ).withName("Stow")
        );
    }

    protected Command aimSub() {
        return loggedCmd(
            StemCommands.moveTo(
                stem,
                StemPosition.STARTING
            ).withName("AimSub")
        );
    }

    protected Command autoShoot() {
        return loggedCmd(
            Commands.parallel(
                new AutoSwerveTargetSpeakerCmd(swerve, vision::getLatestPoseWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(
                    stem,
                    AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                    true,
                    vision::getLatestPoseWithFallback,
                    swerve::getChassisSpeed
                ).finallyDo(() -> logAutoEvent("Stem Targeting", "Done")),
                UmbrellaCommands.waitUntilSpunUp(umbrella, kControls.AUTO_SHOOTER_RPM)
            ).andThen(
                feedShooter()
                    .finallyDo(() -> logAutoEvent("Shooting", "Done"))
            ).withName("AutoShoot")
        );
    }

    protected Command autoShootBegining() {
        return loggedCmd(
            Commands.parallel(
                new AutoSwerveTargetSpeakerCmd(swerve, vision::getLatestPoseWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(
                    stem,
                    AimSolveStrategy.STATIONARY_PIVOT,
                    true,
                    vision::getLatestPoseWithFallback,
                    swerve::getChassisSpeed
                ).finallyDo(() -> logAutoEvent("Stem Targeting", "Done")),
                UmbrellaCommands.waitUntilSpunUp(umbrella, kControls.AUTO_SHOOTER_RPM, 0.4)
            ).andThen(
                feedShooter()
                    .finallyDo(() -> logAutoEvent("Shooting", "Done"))
            ).withName("AutoShootRetract")
        );
    }

    protected Command feedShooter() {
        return loggedCmd(
            UmbrellaCommands.shoot(umbrella, () -> kControls.AUTO_SHOOTER_RPM)
                .withTimeout(0.15).withName("FeedShooter")
        );
    }

    protected Command spinnupShooter() {
        return loggedCmd(
            UmbrellaCommands.spinupShooter(umbrella, kControls.AUTO_SHOOTER_RPM, ShooterSpinupReason.Idle)
                .withName("Spinnupshooter")
        );
    }

}
