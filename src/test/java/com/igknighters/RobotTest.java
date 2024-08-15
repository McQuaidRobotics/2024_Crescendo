package com.igknighters;

import org.junit.jupiter.api.Test;

import choreo.Choreo;
import choreo.ChoreoAutoLoop;
import choreo.ChoreoAutoTrajectory;
import choreo.trajectory.ChoreoTrajectory;

import com.igknighters.constants.RobotConfig.RobotID;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class RobotTest {

    @Test
    public void testRobotSetup() {
        for (RobotID id : RobotID.values()) {
            if (id == RobotID.Unlabeled) {
                continue;
            }

            new Robot(id).close();

            System.gc();
        }
    }

    @Test
    public void testAuto() {
        final Robot robot = new Robot(RobotID.UNIT_TEST);

        final ChoreoTrajectory traj = Choreo.getTrajectory("TEST").get();

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        final double translationTolerance = 0.2;

        Swerve swerve = robot.allSubsystems.swerve.get();
        robot.localizer.resetOdometry(traj.getInitialPose(), swerve.getModulePositions());
        swerve.setYaw(traj.getInitialPose().getRotation());

        robot.autoManager.addAutoRoutine(
            "TestAuto",
            factory -> {
                ChoreoAutoLoop loop = factory.newLoop();
                ChoreoAutoTrajectory aTraj = factory.traj(traj, loop);

                loop.enabled().onTrue(aTraj.cmd());

                return loop.cmd().withName("TestAuto");
            }
        );
        robot.autoManager.choose("TestAuto");

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);

        robot.withAutonomousPeriodicTest(robo -> {
            boolean isFinished = robo.localizer.pose()
                    .getTranslation()
                    .getDistance(traj.getFinalPose().getTranslation()) < translationTolerance;

            if (isFinished) {
                robo.finishUnitTestRobot();
            } else if (robo.getElapsedTime() > 1.5) {
                throw new RuntimeException(
                        "Auto took to long, ended at "
                                + robo.localizer.pose().toString());
            }
        });

        robot.runTest(3);

        robot.close();

        System.gc();
    }

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
