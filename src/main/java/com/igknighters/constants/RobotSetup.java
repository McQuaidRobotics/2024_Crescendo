package com.igknighters.constants;

import java.util.Map;

import com.igknighters.subsystems.Resources.Subsystems;
import com.igknighters.util.BootupLogger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotSetup {

    // to make these only be constructed once each instead of once per enum theyre
    // stored in
    public enum RobotConstID {
        YIN(0), YANG(1);

        public final int value;

        RobotConstID(int value) {
            this.value = value;
        }
    }

    public enum RobotID {
        RobotA("Yin",
                Subsystems.list(Subsystems.Swerve), // can be constructed with enums
                RobotConstID.YIN),

        RobotB("Yang",
                Subsystems.list("Swerve"), // can be constructed with strings(pascal case)
                RobotConstID.YANG),

        ChargedUp("Ammonia",
                Subsystems.list(Subsystems.Swerve),
                RobotConstID.YIN),

        Citrus("Citrus Circus",
                Subsystems.list(Subsystems.Swerve),
                RobotConstID.YIN),

        TestBoard("testBoard(yin)", Subsystems.all(), RobotConstID.YIN),

        Simulation("simulation(yin)", Subsystems.all(), RobotConstID.YIN),

        // this will never be used as if this is hit an error will already have been thrown
        Unlabeled("", Subsystems.none(), RobotConstID.YANG);

        public final String name;
        public final Subsystems[] subsystems;
        public final RobotConstID constID;

        RobotID(String name, Subsystems[] subsystems, RobotConstID constants) {
            this.name = name;
            this.subsystems = subsystems;
            this.constID = constants;
        }
    }

    private static final Map<String, RobotID> serialToID = Map.of(
            "0306adcf", RobotID.TestBoard,
            "0306adf3", RobotID.TestBoard,
            "ffffffff", RobotID.Simulation,
            "aaaaaaaa", RobotID.RobotA,
            "bbbbbbbb", RobotID.RobotB,
            "03260af0", RobotID.ChargedUp,
            "03260abb", RobotID.ChargedUp);

    private static RobotID currentID = RobotID.Unlabeled;

    public static RobotID getRobotID() {
        if (currentID == RobotID.Unlabeled) {
            String currentSerialNum;
            if (RobotBase.isReal()) {
                currentSerialNum = RobotController.getSerialNumber();
                currentSerialNum = currentSerialNum.toLowerCase();
            } else {
                currentSerialNum = "ffffffff";
            }
            if (serialToID.containsKey(currentSerialNum)) {
                currentID = serialToID.get(currentSerialNum);
            } else {
                throw new RuntimeException("Robot ID not found, " + currentSerialNum + " not in serialToID map");
            }
            BootupLogger.BootupLog("Robot Name: " + currentID.name);
        }
        return currentID;
    }
}
