package com.igknighters.commands.autos;

import java.util.function.Supplier;

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
import com.igknighters.subsystems.vision.Vision;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Monologue;

public class AutoCommands {
    protected final Swerve swerve;
    protected final Stem stem;
    protected final Umbrella umbrella;
    protected final Vision vision;
    protected final Localizer localizer;

    private final Supplier<Pose2d> visionPoseSupplierWithFallback;

    protected AutoCommands(Swerve swerve, Stem stem, Umbrella umbrella, Vision vision, Localizer localizer) {
        this.swerve = swerve;
        this.stem = stem;
        this.umbrella = umbrella;
        this.vision = vision;
        this.localizer = localizer;
        visionPoseSupplierWithFallback = () -> localizer.visionPose(0.065);
    }

    protected void logAutoEvent(String name, String event) {
        String msg = "[Auto] Command " + name + " " + event;
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
                UmbrellaCommands.intakeWhileIdleShooter(umbrella, UmbrellaCommands::defaultIdleRPM)
                    .until(() -> umbrella.holdingGamepiece())
            ).withName("IntakeGamepieceNoStow")
        );
    }

    protected Command aimStem(AutoTrajectory traj) {
        return loggedCmd(
            StemCommands.aimAtSpeaker(
                stem,
                AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                false,
                () -> traj.getFinalPose().orElseThrow(),
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
                visionPoseSupplierWithFallback,
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

    protected Command autoShoot(AutoRoutine routine) {
        return loggedCmd(
            Commands.parallel(
                new AutoSwerveTargetSpeakerCmd(swerve, visionPoseSupplierWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(
                    stem,
                    AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                    true,
                    visionPoseSupplierWithFallback,
                    swerve::getChassisSpeed
                ).finallyDo(() -> logAutoEvent("Stem Targeting", "Done")),
                UmbrellaCommands.waitUntilSpunUp(umbrella, kControls.AUTO_SHOOTER_RPM)
            ).andThen(
                feedShooter()
                    .finallyDo(() -> logAutoEvent("Shooting", "Done"))
            ).onlyWhile(yeGp(routine)).withName("AutoShoot")
        );
    }

    protected Command autoShootThenTraj(AutoRoutine routine, AutoTrajectory traj) {
        return loggedCmd(
            Commands.parallel(
                new AutoSwerveTargetSpeakerCmd(swerve, visionPoseSupplierWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(
                    stem,
                    AimSolveStrategy.STATIONARY_PIVOT_TELESCOPE_EXTEND,
                    true,
                    visionPoseSupplierWithFallback,
                    swerve::getChassisSpeed
                ).finallyDo(() -> logAutoEvent("Stem Targeting", "Done"))
            ).andThen(
                feedShooter()
                    .finallyDo(() -> logAutoEvent("Shooting", "Done")),
                new ScheduleCommand(traj.cmd())
            ).withName("AutoShootThenTraj")
        );
    }

    protected Command autoShootBegining() {
        return loggedCmd(
            Commands.parallel(
                new AutoSwerveTargetSpeakerCmd(swerve, visionPoseSupplierWithFallback)
                    .finallyDo(() -> logAutoEvent("SwerveTargeting", "Done")),
                StemCommands.aimAtSpeaker(
                    stem,
                    AimSolveStrategy.STATIONARY_PIVOT,
                    true,
                    visionPoseSupplierWithFallback,
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
                .withTimeout(0.4)
                .withName("FeedShooter")
        );
    }

    protected Command spinnupShooter() {
        return loggedCmd(
            UmbrellaCommands.spinupShooter(umbrella, kControls.AUTO_SHOOTER_RPM)
                .withName("Spinnupshooter")
        );
    }

    protected Trigger yeGp(AutoRoutine routine) {
        Debouncer debouncer = new Debouncer(0.3, DebounceType.kFalling);
        return new Trigger(routine.loop(), () -> debouncer.calculate(umbrella.holdingGamepiece()));
    }

    protected Trigger noGp(AutoRoutine routine) {
        return yeGp(routine).negate();
    }
}
