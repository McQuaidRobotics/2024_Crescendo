package com.igknighters;

import com.igknighters.commands.swerve.TeleopSwerve;
import com.igknighters.constants.ConstValues;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    static {
        com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);
    }

    private final CommandXboxController driveController = new CommandXboxController(0);
    @SuppressWarnings("unused")
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final Swerve swerve = new Swerve();

    public RobotContainer() {

        DriverStation.silenceJoystickConnectionWarning(true);

        configureDriverBindings();
        configureOperatorBindings();

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        driveController::getLeftY,
                        driveController::getLeftX,
                        driveController::getRightX));
    }

    private void configureDriverBindings() {

        // Center Buttons
        driveController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));
    }

    private void configureOperatorBindings() {
    }
}
