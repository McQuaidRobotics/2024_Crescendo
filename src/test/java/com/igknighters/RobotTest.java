package com.igknighters;

import org.junit.jupiter.api.Test;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

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

        final Optional<Trajectory<SwerveSample>> optTraj = Choreo.loadTrajectory("TEST");
        final Trajectory<SwerveSample> traj = optTraj.orElseThrow();

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        final double translationTolerance = 0.2;

        Swerve swerve = robot.allSubsystems.swerve.get();
        robot.localizer.reset(traj.getInitialPose(false));
        swerve.setYaw(traj.getInitialPose(false).getRotation());

        robot.autoChooser.addAutoRoutine(
            "TestAuto",
            factory -> {
                AutoRoutine routine = factory.newRoutine("TestAutoLoop");
                AutoTrajectory aTraj = factory.trajectory(traj, routine);

                routine.enabled().onTrue(aTraj.cmd());

                return routine;
            }
        );
        // robot.autoChooser.choose("TestAuto");

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);

        robot.withAutonomousPeriodicTest(robo -> {
            boolean isFinished = robo.localizer.pose()
                    .getTranslation()
                    .getDistance(traj.getFinalPose(false).getTranslation()) < translationTolerance;

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
