package com.igknighters.commands.umbrella;

import com.igknighters.subsystems.umbrella.Umbrella;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class UmbrellaCommands {

    public static Command testUmbrella(Umbrella umbrella, DoubleSupplier intakeSupplier, DoubleSupplier shooterSupplier) {
        return umbrella.run(
            () -> {
                umbrella.spinupShooterToRotSpeed(shooterSupplier.getAsDouble() * 3000);
                umbrella.runIntakeAt(intakeSupplier.getAsDouble());
            }
        );
    }
}
