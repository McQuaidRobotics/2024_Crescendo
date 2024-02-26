package com.igknighters.commands.umbrella;


import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.subsystems.umbrella.Umbrella.ShooterSpinupReason;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
     * @param rpm      The target speed
     * @return A command to be scheduled
     */
    public static Command spinupShooter(Umbrella umbrella, double rpm, ShooterSpinupReason reason) {
        return umbrella.runOnce(() -> {
            umbrella.spinupShooterToRPM(rpm);
            umbrella.pushSpinupReason(reason);
        }).withName("Spinup Shooter");
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
     * Will shoot any held game piece, otherwise will do nothing
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command shoot(Umbrella umbrella) {
        return umbrella.run(
                () -> {
                    umbrella.spinupShooter(umbrella.getShooterTargetSpeed());
                    umbrella.runIntakeAt(-1.0, true);
                })
                .withTimeout(0.75)
                // .until(umbrella::notHoldingGamepiece)
                .unless(() -> umbrella.getShooterSpeed() < 30.0)
                .finallyDo(umbrella::stopAll)
                .withName("Shoot");
    }

    /**
     * Will spin the intake inwards until a game piece is held
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command intake(Umbrella umbrella) {
        return umbrella.runEnd(
                () -> umbrella.runIntakeAt(-0.9, false),
                umbrella::stopAll).until(umbrella::holdingGamepiece)
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
     * A command primarily for testing, will run the intake and shooter at the
     * provided values in SmartDashboard
     * 
     * @param umbrella The umbrella subsystem
     * @return A command to be scheduled
     */
    public static Command spinUmbrellaBoth(Umbrella umbrella) {
        SmartDashboard.putNumber("IntakePercent", 0.0);
        SmartDashboard.putNumber("RPMumbrella", 0.0);
        return umbrella.run(() -> {
            umbrella.runIntakeAt(
                    SmartDashboard.getNumber("IntakePercent", 0));
            umbrella.spinupShooterToRPM(
                    SmartDashboard.getNumber("RPMumbrella", 0));
        }).withName("Spin Umbrella Both");
    }
}
