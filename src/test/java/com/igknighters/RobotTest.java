package com.igknighters;

import org.junit.jupiter.api.Test;

// import com.igknighters.commands.autos.Autos;
// import com.igknighters.util.RobotExtension.Robo;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotConfig;
import com.igknighters.constants.RobotConfig.RobotID;

// import edu.wpi.first.hal.AllianceStationID;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class RobotTest {

    @Test
    public void testRobotSetup() {
        for (RobotID id : RobotID.values()) {
            if (id == RobotID.Unlabeled) {
                continue;
            }

            RobotConfig.testOverrideRobotID(id);

            com.igknighters.constants.ConstantHelper.applyRoboConst(ConstValues.class);

            new Robot().close();

            System.gc();
        }
    }

    // @Test
    // public void testAuto(@Robo Robot robot) {
    //     RobotSetup.testOverrideRobotID(RobotID.SIM_CRASH);
    //     Pose2d desiredEndPose = new Pose2d(
    //             new Translation2d(3.0, 7.0),
    //             new Rotation2d());

    //     DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    //     // meters
    //     final double translationTolerance = 0.2;

    //     Autos.setAutoOverrideTest(new ProxyCommand(() -> new PathPlannerAuto("1 Meter Auto")));
    //     DriverStationSim.setAutonomous(true);
    //     DriverStationSim.setEnabled(true);

    //     robot.withAutonomousPeriodicTest(robo -> {
    //         boolean isFinished = robo.localizer.pose()
    //                 .getTranslation()
    //                 .getDistance(desiredEndPose.getTranslation()) < translationTolerance;

    //         if (isFinished) {
    //             robo.finishUnitTestRobot();
    //         } else if (robo.getElapsedTime() > 2.5) {
    //             throw new RuntimeException(
    //                     "Auto took to long, ended at "
    //                             + robo.localizer.pose().toString());
    //         }
    //     });

    //     robot.runTest(3);

    //     System.gc();
    // }

    // @Test
    // public void testShooter(@Robo Robot robot) {
    //     RobotSetup.testOverrideRobotID(RobotID.SIM_CRASH);

    //     DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    //     DriverStationSim.setEnabled(true);

    //     robot.withTeleopInitTest(robo -> {
    //         var umbrella = robo.getAllSubsystemsForTest().umbrella
    //                 .get();

    //         umbrella.run(() -> umbrella.spinupShooterToRPM(3000));
    //     })
    //             .withTeleopPeriodicTest(robo -> {
    //                 var umbrella = robo.getAllSubsystemsForTest().umbrella
    //                         .get();

    //                 if (umbrella.isShooterAtSpeed()) {
    //                     robo.finishUnitTestRobot();
    //                 }
    //             });

    //     robot.runTest(3);
    // }
}
