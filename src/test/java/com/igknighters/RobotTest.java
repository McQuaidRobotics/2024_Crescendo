package com.igknighters;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.RobotSetup.RobotID;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class RobotTest {

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        GlobalState.setUnitTest(true);
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
}
