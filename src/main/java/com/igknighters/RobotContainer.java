package com.igknighters;

import com.igknighters.commands.swerve.TeleopSwerve;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.SubsystemResources.AllSubsystems;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

    static {
        com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);
    }

    private static final DriverController driverController = new DriverController(0);
    private static final OperatorController operatorController = new OperatorController(1);
    private static final TestingController testingController = new TestingController(3);

    private final AllSubsystems allSubsystems = new AllSubsystems(RobotSetup.getRobotID().subsystems);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(ConstValues.DEBUG);

        driverController.assignButtons(allSubsystems);
        operatorController.assignButtons(allSubsystems);
        testingController.assignButtons(allSubsystems);

        if (allSubsystems.swerve.isPresent()) {
            var swerve = allSubsystems.swerve.get();
            swerve.setDefaultCommand(
                    new TeleopSwerve(
                            swerve,
                            driverController.leftStickY(),
                            driverController.leftStickX(),
                            driverController.rightStickX()));
        }
    }

    // private void configureDriverBindings() {

    // // Center Buttons
    // driveController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));
    // }

    // private void configureOperatorBindings() {
    // }
}
