package com.igknighters.commands.umbrella;

import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.Umbrella;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class UmbrellaCommands {
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
     * Spins up the shooter to a certain speed
     * 
     * @param umbrella The umbrella subsystem
     * @param rpm      The target speed
     * @return A command to be scheduled
     */
    public static Command spinupShooter(Umbrella umbrella, double rpm) {
        return umbrella.runOnce(() -> umbrella.spinupShooterToRPM(rpm));
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
                    if (!umbrella.holdingGamepiece()) {
                        return Commands.none();
                    }
                    return umbrella.run(
                            () -> {
                                umbrella.spinupShooter(umbrella.getShooterTargetSpeed());
                                umbrella.turnIntakeBy(-1.0);
                            }).until(
                                    () -> !umbrella.holdingGamepiece())
                            .andThen(
                                    () -> {
                                        umbrella.runIntakeAt(0.0);
                                        umbrella.spinupShooterToRPM(0);
                                    });
                });
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
                () -> umbrella.runIntakeAt(0.0)).until(() -> umbrella.holdingGamepiece());
    }

    public static Command expell(Umbrella umbrella) {
        return umbrella.runEnd(
                () -> umbrella.runIntakeAt(1.0),
                () -> umbrella.runIntakeAt(0.0));
    }

    public static Command feed(Umbrella umbrella) {
        return umbrella.runOnce(
                () -> umbrella.turnIntakeBy(0.5)).repeatedly()
                .until(() -> umbrella.holdingGamepiece());
    }

    public static Command spinUmbrellaBoth(Umbrella umbrella) {
        SmartDashboard.putNumber("IntakePercent", 0.0);
        SmartDashboard.putNumber("RPMumbrella", 0.0);
        return umbrella.run(() -> {
            umbrella.runIntakeAt(
                    NetworkTableInstance
                            .getDefault()
                            .getTable("/SmartDashboard")
                            .getEntry("IntakePercent")
                            .getDouble(0.0));
            umbrella.spinupShooterToRPM(
                    NetworkTableInstance
                            .getDefault()
                            .getTable("/SmartDashboard")
                            .getEntry("RPMumbrella")
                            .getDouble(0.0));
        });
    }
}
