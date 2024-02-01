package com.igknighters.commands.umbrella;

import com.igknighters.subsystems.umbrella.Umbrella;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class UmbrellaCommands {

    public static Command testUmbrella(Umbrella umbrella, DoubleSupplier intakeSupplier, DoubleSupplier shooterSupplier) {
        return umbrella.run(
            () -> {
                umbrella.spinupShooterToRotSpeed(shooterSupplier.getAsDouble() * 3000);
                umbrella.runIntakeAt(intakeSupplier.getAsDouble());
            }
        );
    }

    public static Command spinUmbrella(Umbrella umbrella) {
        SmartDashboard.putNumber("RPMumbrella", 0);
        Timer timer = new Timer();

        return new FunctionalCommand(
            () -> timer.restart(),
            () -> {
                umbrella.spinupShooterToRotSpeed(
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


}
