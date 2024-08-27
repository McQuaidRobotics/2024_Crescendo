package com.igknighters.constants;

import java.util.List;
import java.util.Map;

import com.igknighters.constants.ConstantHelper.RobotConstID;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.util.logging.BootupLogger;
import com.igknighters.Robot;
import edu.wpi.first.wpilibj.RobotController;
import monologue.Monologue;

public class RobotConfig {

    public enum RobotID {
        CRASH(Subsystems.all(), RobotConstID.CRASH),

        BURN(Subsystems.list(Subsystems.Swerve),
                RobotConstID.BURN),

        SIM_CRASH(Subsystems.all(), RobotConstID.CRASH),
        SIM_BURN(Subsystems.none(), RobotConstID.BURN),

        TestBoard("testBoard(crash)", Subsystems.list(Subsystems.Umbrella), RobotConstID.CRASH),

        UNIT_TEST(Subsystems.all(), RobotConstID.CRASH),

        Unlabeled("", Subsystems.none(), RobotConstID.BURN);

        public final String name;
        public final Subsystems[] subsystems;
        public final RobotConstID constID;

        RobotID(String name, Subsystems[] subsystems, RobotConstID constants) {
            this.name = name;
            this.subsystems = subsystems;
            this.constID = constants;
        }

        RobotID(Subsystems[] subsystems, RobotConstID constants) {
            this.name = this.name();
            this.subsystems = subsystems;
            this.constID = constants;
        }

        public boolean isSubsystemEnabled(Subsystems sub) {
            for (Subsystems enabledSub : subsystems) {
                if (enabledSub == sub) {
                    return true;
                }
            }
            return false;
        }
    }

    /**
     * If there are duplicate serial entries the tests will fail!!!!
     */
    private static final Map<String, RobotID> serialToID = Map.of(
            "0306adcf", RobotID.TestBoard,
            "0306adf3", RobotID.TestBoard,
            "ffffffff", RobotID.SIM_CRASH,
            "aaaaaaaa", RobotID.CRASH,
            "03260af0", RobotID.BURN,
            "03260abb", RobotID.CRASH,
            "0306adb6", RobotID.TestBoard,
            "032b4b20", RobotID.CRASH);

    private static RobotID currentID = RobotID.Unlabeled;

    public static RobotID getRobotID() {
        if (currentID == RobotID.Unlabeled) {
            String currentSerialNum;
            if (Robot.isReal()) {
                currentSerialNum = RobotController.getSerialNumber();
                if (currentSerialNum == null) {
                    throw new RuntimeException("Tried loading robot ID before advantage-kit was ready!");
                }
                currentSerialNum = currentSerialNum.toLowerCase();
            } else {
                currentSerialNum = "ffffffff";
            }
            if (serialToID.containsKey(currentSerialNum)) {
                currentID = serialToID.get(currentSerialNum);
            } else {
                throw new RuntimeException("Robot ID not found, " + currentSerialNum + " not in serialToID map");
            }
            BootupLogger.bootupLog("Robot Name: " + currentID.name);
            Monologue.log("RobotConfig/RobotId", currentID.name());
            Monologue.log(
                    "RobotConfig/EnabledSubsystems",
                    List.of(currentID.subsystems)
                            .stream()
                            .map(sub -> sub.name())
                            .reduce("", (acc, sub) -> acc + sub + ", "));
            Monologue.log("RobotConfig/ConstantsID", currentID.constID.name());
        }
        return currentID;
    }
}
