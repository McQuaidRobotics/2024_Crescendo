package com.igknighters;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.autos.AutosCmdRegister;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

            if (RobotBase.isReal()) {
                swerve.setDefaultCommand(
                        new com.igknighters.commands.swerve.TeleopSwerve(
                                swerve,
                                driverController.leftStickY(),
                                driverController.leftStickX(),
                                driverController.rightStickX()));
            } else {
                swerve.setDefaultCommand(
                        new com.igknighters.commands.swerve.TeleopSwerveSim(
                                swerve,
                                driverController.leftStickY(),
                                driverController.leftStickX(),
                                driverController.rightStickX()));
            }

            // swerve.setDefaultCommand(
            //         new com.igknighters.commands.swerve.TeleopSwerveAbsRot(
            //                 swerve,
            //                 driverController.leftStickX(),
            //                 driverController.leftStickY(),
            //                 driverController.rightStickX(),
            //                 driverController.rightStickY()
            //         ));

            setupAutos();
        }
    }

    public void setupAutos() {
        AutosCmdRegister.registerCommands(allSubsystems);

        if (!allSubsystems.swerve.isPresent())
            return;
        var swerve = allSubsystems.swerve.get();
        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetOdometry,
                swerve::getChassisSpeeds,
                chassisSpeeds -> swerve.driveChassisSpeeds(
                    chassisSpeeds, false, RobotBase.isReal()
                ),
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_DRIVE_VELOCITY,
                        kSwerve.DRIVEBASE_RADIUS,
                        new ReplanningConfig(
                                true,
                                true)),
                () -> {
                    if (DriverStation.getAlliance().isPresent() 
                    && DriverStation.getAlliance().get() == Alliance.Blue) {
                        return false;
                    }
                    else if (DriverStation.getAlliance().isPresent() 
                    && DriverStation.getAlliance().get() == Alliance.Red) {
                        return true;
                    }
                    else return false; //Default path for blue alliance side
                },
                swerve);
    }
}
