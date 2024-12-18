package com.igknighters.commands.umbrella;

import java.util.function.DoubleSupplier;

import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.Umbrella;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class UmbrellaCommands {
    /**
     * Commands the shooter to not output any power
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command stopShooter(Umbrella umbrella) {
        return umbrella.runOnce(() -> umbrella.spinupShooterToRPM(0))
                .withName("Stop Shooter");
    }

    /**
     * Spins up the shooter to a certain speed
     * 
     * @param umbrella The umbrella subsystem
     * @param rpmSup      A supplier for the target speed in rotations per minute
     * @return A command to be scheduled
     */
    public static Command spinupShooter(Umbrella umbrella, DoubleSupplier rpmSup) {
        return umbrella.run(() -> {
            umbrella.spinupShooterToRPM(rpmSup.getAsDouble());
        }).withName("Spinup Shooter");
    }

    /**
     * Spins up the shooter to a certain speed
     * 
     * @param umbrella The umbrella subsystem
     * @param rpm      The target speed in rotations per minute
     * @return A command to be scheduled
     */
    public static Command spinupShooter(Umbrella umbrella, double rpm) {
        return spinupShooter(umbrella, () -> rpm);
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
                        () -> umbrella.isShooterAtSpeed(tolerance))
                .withName("Wait Until Spun Up");
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
     * Runs the intake and the shooter, feeding notes through the umbrella
     * 
     * @param umbrella The umbrella subsystem
     * @param rpm The rpm to spin the shooter up to
     * @return A command to be scheduled
     */
    public static Command shoot(Umbrella umbrella, DoubleSupplier rpmSupplier) {
        return umbrella.run(
                () -> {
                    umbrella.runIntakeAt(-1.0, true);
                    umbrella.spinupShooterToRPM(rpmSupplier.getAsDouble());
                }).withTimeout(0.5).withName("Shoot");
    }

    /**
     * Will spin the intake inwards until a game piece is held
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command intake(Umbrella umbrella) {
        return umbrella.runEnd(
                () -> umbrella.runIntakeAt(-0.7, false),
                () -> umbrella.runIntakeAt(0.0, false)).until(umbrella::holdingGamepiece)
                .withName("Intake");
    }

    /**
     * Will spin the intake outwards until canceled
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command expell(Umbrella umbrella) {
        return umbrella.runEnd(
                () -> umbrella.runIntakeAt(1.0, true),
                umbrella::stopAll)
                .withName("Expell");
    }

    /**
     * Returns the target RPM of the shoote rwhile idling based off the current mode
     * 
     * @return The target RPM
     */
    public static double defaultIdleRPM() {
        return Robot.isDemo() ? 0.0
                : DriverStation.isAutonomousEnabled() ? kControls.AUTO_SHOOTER_RPM : kControls.SHOOTER_IDLE_RPM;
    }

    /**
     * Will spin the shooter up to its idle speed,
     * in demo mode the shooter will be stopped
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command idleShooter(Umbrella umbrella, DoubleSupplier rpmSupplier) {
        return umbrella.run(() -> {
            umbrella.runIntakeAt(0.0);
            umbrella.spinupShooterToRPM(rpmSupplier.getAsDouble());
        }).withName("IdleIntake");
    }

    /**
     * Will spin the intake inwards until a game piece is held, and spin the shooter up to its idle speed
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command intakeWhileIdleShooter(Umbrella umbrella, DoubleSupplier rpmSupplier) {
        return umbrella.runEnd(
                () -> {
                    umbrella.runIntakeAt(-0.95, false);
                    umbrella.spinupShooterToRPM(rpmSupplier.getAsDouble());
                },
                () -> umbrella.runIntakeAt(0.0, false)
        ).until(umbrella::holdingGamepiece)
        .withName("IntakeWWhileSpinupShooter");
    }
}
