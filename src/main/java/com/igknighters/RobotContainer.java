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

    private final DriverController driverController;
    private final OperatorController operatorController;
    private final TestingController testingController;

    private static AllSubsystems allSubsystems;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(ConstValues.DEBUG);

        driverController = new DriverController(0);
        operatorController = new OperatorController(1);
        testingController = new TestingController(3);

        allSubsystems = new AllSubsystems(RobotSetup.getRobotID().subsystems);

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
}
