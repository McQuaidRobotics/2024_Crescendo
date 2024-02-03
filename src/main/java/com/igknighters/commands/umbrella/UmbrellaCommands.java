package com.igknighters.commands.umbrella;

import com.igknighters.GlobalState;
import com.igknighters.GlobalState.GamepieceState;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.Umbrella;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;


public class UmbrellaCommands {

    public static Command testUmbrella(Umbrella umbrella, DoubleSupplier intakeSupplier,
            DoubleSupplier shooterSupplier) {
        return umbrella.run(
                () -> {
                    umbrella.spinupShooterToRPM(shooterSupplier.getAsDouble() * 3000);
                    umbrella.runIntakeAt(intakeSupplier.getAsDouble());
                });
    }

    /**
     * Commands the shooter to not output any power
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command stopShooter(Umbrella umbrella) {
        return umbrella.runOnce(() -> umbrella.spinupShooterToRPM(0));
    }

    /**
     * A command that waits until the shooter is spun up to a certain speed
     * 
     * @param umbrella  The umbrella subsystem
     * @param rpm       The target speed
     * @param tolerance The acceptable error in percent of max speed
     * @return A command to be scheduled
     */
    public static Command waitUntilSpunUp(Umbrella umbrella, double rpm, double tolerance) {
        return umbrella.run(
                () -> umbrella.spinupShooterToRPM(rpm)).until(
                        () -> umbrella.isShooterAtSpeed(tolerance));
    }

    /**
     * A command that waits until the shooter is spun up to a certain speed
     * 
     * @param umbrella The umbrella subsystem
     * @param rpm      The target speed
     * @return A command to be scheduled
     */
    public static Command waitUntilSpunUp(Umbrella umbrella, double rpm) {
        return waitUntilSpunUp(umbrella, rpm, kShooter.DEFAULT_TOLERANCE);
    }

    /**
     * Will shoot any held game piece, otherwise will do nothing
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command shoot(Umbrella umbrella) {
        return umbrella.defer(
            () -> {
                if (
                    GlobalState.getGamePieceState() == GamepieceState.None
                    || umbrella.getShooterTargetSpeed() < kShooter.MIN_SHOOT_SPEED)
                {
                    DriverStation.reportWarning("Robot state is not fit for shooting", false);
                    return Commands.none();
                }
                var cmd = umbrella.run(
                    () -> {
                        umbrella.spinupShooter(umbrella.getShooterTargetSpeed());
                        umbrella.runIntakeAt(-1.0);
                    }
                ).until(
                    () -> !umbrella.isExitBeamBroken() && !umbrella.isEntranceBeamBroken()
                ).andThen(
                    () -> {
                        umbrella.runIntakeAt(0.0);
                        umbrella.spinupShooterToRPM(0);
                        GlobalState.setHasGamePiece(GamepieceState.None);
                    }
                );

                if (GlobalState.getGamePieceState() == GamepieceState.Held) {
                    return umbrella.getDefaultCommand()
                        .until(() -> GlobalState.getGamePieceState() == GamepieceState.Confirmed)
                        .andThen(cmd);
                }

                return cmd;
            }
        );
    }

    public static Command spinUmbrella(Umbrella umbrella) {
        SmartDashboard.putNumber("RPMumbrella", 0);
        Timer timer = new Timer();

        return new FunctionalCommand(
            () -> timer.restart(),
            () -> {
                umbrella.spinupShooterToRPM(
                     NetworkTableInstance
                    .getDefault()
                    .getTable("/SmartDashboard")
                    .getEntry("RPMumbrella")
                    .getDouble(0.0)
                );
                if (umbrella.isShooterAtSpeed(0.02)) {
                    SmartDashboard.putNumber("WindupTime", timer.get());
                    timer.stop();
                }
            },
            bool -> {},
            () -> false,
            umbrella
        );
    }


    /**
     * Ensures the gamepiece is not touching the shooter wheels
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command confirm(Umbrella umbrella) {
        return umbrella.run(
            () -> {
                GamepieceState state = GlobalState.getGamePieceState();

                if (state == GamepieceState.Confirmed) {
                    return;
                }

                if ((umbrella.isExitBeamBroken() || umbrella.isEntranceBeamBroken()) && state == GamepieceState.None) {
                    GlobalState.setHasGamePiece(GamepieceState.Held);
                    state = GamepieceState.Held;
                }

                if (umbrella.isExitBeamBroken()) {
                    umbrella.turnIntakeBy(-Units.inchesToMeters(0.1));
                } else {
                    GlobalState.setHasGamePiece(GamepieceState.Confirmed);
                    state = GamepieceState.Confirmed;
                }
            }
        ).until(
            () -> GlobalState.getGamePieceState() == GamepieceState.Confirmed
        );
    }

    /**
     * A command that waits until the intake is not holding a game piece
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command intake(Umbrella umbrella) {
        return umbrella.runEnd(
            () -> umbrella.runIntakeAt(-1.0),
            () -> umbrella.runIntakeAt(0.0)
        ).until(
            () -> umbrella.isEntranceBeamBroken() || umbrella.isExitBeamBroken()
        );
    }
}
