package com.igknighters;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.swerve.Swerve;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.autos.Autos;
import com.igknighters.commands.autos.AutosCmdRegister;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBase;
import com.igknighters.commands.umbrella.UmbrellaCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotContainer {

    private final DriverController driverController;
    private final OperatorController operatorController;
    private final TestingController testingController;

    private final AllSubsystems allSubsystems;

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

            swerve.setDefaultCommand(new TeleopSwerveBase.TeleopSwerveOmni(swerve, driverController));

            setupAutos(swerve);

            Autos.createSendableChooser(swerve);
        }

        if (allSubsystems.umbrella.isPresent()){
            var umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(UmbrellaCommands.spinUmbrella(umbrella));
        }
    }

    private void setupAutos(Swerve swerve) {

        if (AutoBuilder.isConfigured() && GlobalState.isUnitTest()) {
            // this code can be run multiple times during unit tests,
            // because of AutoBuilder once paradigm this causes a crash
            return;
        }

        AutosCmdRegister.registerCommands(allSubsystems);

        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetOdometry,
                swerve::getChassisSpeed,
                chassisSpeeds -> swerve.driveChassisSpeeds(
                        chassisSpeeds, false),
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_DRIVE_VELOCITY,
                        kSwerve.DRIVEBASE_RADIUS,
                        kAuto.DYNAMIC_REPLANNING_CONFIG),
                () -> {
                    if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Blue) {
                        return false;
                    } else if (DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red) {
                        return true;
                    } else
                        return false; // Default path for blue alliance side
                },
                swerve);

        GlobalState.onceInitAutoChooser(swerve);
    }

    AllSubsystems getAllSubsystemsForTest() {
        if (!GlobalState.isUnitTest()) {
            throw new RuntimeException("This method should only be called in unit tests");
        }
        return allSubsystems;
    }
}
