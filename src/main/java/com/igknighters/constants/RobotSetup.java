package com.igknighters.constants;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.igknighters.SubsystemResources.Subsystems;
import com.igknighters.util.BootupLogger;
import com.igknighters.ConstantHelper.RobotConstID;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotSetup {

    public enum RobotID {
        CRASH(Subsystems.list(Subsystems.Swerve),
                RobotConstID.CRASH),

        BURN(Subsystems.list("Swerve"),
                RobotConstID.BURN),

        SIM_CRASH(Subsystems.all(), RobotConstID.CRASH),
        SIM_BURN(Subsystems.none(), RobotConstID.BURN),

        TestBoard("testBoard(yin)", Subsystems.all(), RobotConstID.CRASH),

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
    }

    private static final Map<String, RobotID> serialToID = Map.of(
            "0306adcf", RobotID.TestBoard,
            "0306adf3", RobotID.TestBoard,
            "ffffffff", RobotID.SIM_CRASH,
            "aaaaaaaa", RobotID.CRASH,
            "bbbbbbbb", RobotID.BURN,
            "03260abb", RobotID.CRASH,
            "riobombb", RobotID.SIM_BURN
        );

    private static RobotID currentID = RobotID.Unlabeled;

    public static RobotID getRobotID() {
        if (currentID == RobotID.Unlabeled) {
            String currentSerialNum;
            if (RobotBase.isReal()) {
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
            BootupLogger.BootupLog("Robot Name: " + currentID.name);
        }
        Logger.recordMetadata("RobotId", currentID.name());
        Logger.recordMetadata(
            "EnabledSubsystems",
            List.of(currentID.subsystems)
                .stream()
                .map(sub -> sub.name())
                .reduce("", (acc, sub) -> acc + sub + ", ")
        );
        Logger.recordMetadata("ConstantsID", currentID.constID.name());
        return currentID;
    }
}
