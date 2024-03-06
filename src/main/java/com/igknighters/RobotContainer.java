package com.igknighters;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.commands.autos.AutosCmdRegister;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import monologue.Logged;

public class RobotContainer implements Logged {

    private final DriverController driverController;
    private final OperatorController operatorController;
    private final TestingController testingController;

    private final AllSubsystems allSubsystems;

    @SuppressWarnings("unused")
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(ConstValues.DEBUG || Robot.isSimulation());

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
        }

        if (allSubsystems.stem.isPresent() && ConstValues.DEBUG) {
            var stem = allSubsystems.stem.get();
            stem.setDefaultCommand(stem.run(() -> {
                stem.setStemVolts(
                        testingController.leftStickY(0.1).getAsDouble() * 12.0,
                        (testingController.rightTrigger(true).getAsDouble()
                                - testingController.leftTrigger(true).getAsDouble()) * 6.0,
                        testingController.rightStickY(0.1).getAsDouble() * 12.0);
            }).withName("StemDefaultCommand"));
        }

        double staticSubShotRads = StemSolvers.linearSolvePivotTheta(kTelescope.MIN_METERS,
                Units.degreesToRadians(71.5), Units.inchesToMeters(46.088 + 2.8), FieldConstants.SPEAKER.getZ());
        System.out.println(staticSubShotRads);
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
                chassisSpeeds -> swerve.drive(
                        chassisSpeeds, false),
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_DRIVE_VELOCITY,
                        kSwerve.DRIVEBASE_RADIUS,
                        kAuto.DYNAMIC_REPLANNING_CONFIG),
                AllianceFlip::isRed,
                swerve);

        GlobalState.onceInitAutoChooser(swerve);
    }

    AllSubsystems getAllSubsystemsForTest() {
        // if (!GlobalState.isUnitTest()) {
        // throw new RuntimeException("This method should only be called in unit
        // tests");
        // }
        return allSubsystems;
    }
}
