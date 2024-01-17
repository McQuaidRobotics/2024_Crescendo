package com.igknighters;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.igknighters.commands.autos.Autos;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.RobotSetup.RobotID;
import com.igknighters.util.UnitTestableRobot.Mode;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotTest {

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        GlobalState.setUnitTest(true);
    }

    @AfterEach
    void teardown() {
        HAL.exitMain();
        HAL.shutdown();
        GlobalState.restoreDefaultState();
    }

    @Test
    public void testRobotSetup() {
    for (RobotID id : RobotID.values()) {
            if (id == RobotID.Unlabeled) {
                continue;
            }

            RobotSetup.testOverrideRobotID(id);

            com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);

            new RobotContainer();

            CommandScheduler.getInstance().getActiveButtonLoop().clear();
            CommandScheduler.getInstance().getDefaultButtonLoop().clear();

            System.gc();
        }
    }

    static Command autoCmd;
    @Test
    public void testAuto() {
        RobotSetup.testOverrideRobotID(RobotID.SIM_CRASH);
        Pose2d desiredEndPose = new Pose2d(
                new Translation2d(3.0, 7.0),
                new Rotation2d());

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

        // meters
        final double translationTolerance = 0.2;

        Timer timer = new Timer();
        timer.start();

        Robot.testRobot(Robot::new, (robot) -> {
            if (robot.lastMode != Mode.kAutonomous) {
                autoCmd = new PathPlannerAuto("1 Meter Auto");
                Autos.setAutoOverrideTest(autoCmd);
                DriverStationSim.setAutonomous(true);
                DriverStationSim.setEnabled(true);
            }

            boolean isFinished = GlobalState.getLocalizedPose()
                .getTranslation()
                .getDistance(desiredEndPose.getTranslation())
                < translationTolerance;

            if (isFinished) {
                robot.killUnitTestRobot();
            } else if (timer.hasElapsed(2.5)) {
                throw new RuntimeException(
                        "Auto took to long, ended at "
                                + GlobalState.getLocalizedPose().toString());
            }
        });
    }
}
