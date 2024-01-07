package com.igknighters;

import com.igknighters.commands.swerve.TeleopSwerve;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.autos.AutosCmdRegister;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

    static {
        com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);
    }

    private static final DriverController driverController = new DriverController(0);
    private static final OperatorController operatorController = new OperatorController(1);
    // private static final TestingController testingController = new TestingController(3);

    private final AllSubsystems allSubsystems = new AllSubsystems(RobotSetup.getRobotID().subsystems);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(ConstValues.DEBUG);

        driverController.assignButtons(allSubsystems);
        operatorController.assignButtons(allSubsystems);
        // testingController.assignButtons(allSubsystems);

        if (allSubsystems.swerve.isPresent()) {
            var swerve = allSubsystems.swerve.get();

            // driverController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));

            swerve.setDefaultCommand(
                    new TeleopSwerve(
                            swerve,
                            driverController.leftStickY(),
                            driverController.leftStickX(),
                            driverController.rightStickX()));
        }

        setupAutos();
    }

    // private void configureDriverBindings() {
    //     // Center Buttons
    //     // driverController.start().onTrue(new InstantCommand(() -> swerve.setYaw(0.0)));
    // }

    // private void configureOperatorBindings() {
    // }

    private void setupAutos() {
        AutosCmdRegister.registerCommands(allSubsystems);

        if (!allSubsystems.swerve.isPresent()) return;
        var swerve = allSubsystems.swerve.get();
        AutoBuilder.configureHolonomic(
                swerve::getPose, 
                swerve::resetOdometry, 
                swerve::getChassisSpeeds, 
                swerve::driveRobotRelative, 
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_DRIVE_VELOCITY,
                        kSwerve.DRIVEBASE_RADIUS,
                        new ReplanningConfig(
                                true, 
                                true)
                ),
                swerve);
    }
}
