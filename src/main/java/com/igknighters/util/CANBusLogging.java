package com.igknighters.util;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.igknighters.Robot;

import edu.wpi.first.wpilibj.Timer;

public class CANBusLogging {
    private static final ArrayList<String> loggedBuses = new ArrayList<>();
    private static final String path = "CANBus";
    private static final Timer timer = new Timer();
    private static int index = 0;

    public static void logBus(String busName) {
        if (Robot.isSimulation() || loggedBuses.contains(busName)) {
            return;
        }

        Logger.recordOutput(
            path + "/" + busName + "/" + "isFd",
            CANBus.isNetworkFD(busName)
        );

        timer.start();

        loggedBuses.add(busName);
    }

    public static void run() {
        if (Robot.isSimulation() || loggedBuses.isEmpty()) {
            return;
        }

        if (timer.hasElapsed(5.0)) {
            timer.reset();
        } else {
            return;
        }

        String busName = loggedBuses.get(index);

        var status = CANBus.getStatus(busName);

        String prefix = path + "/" + busName + "/";

        Logger.recordOutput(prefix + "status", status.Status.name());
        Logger.recordOutput(prefix + "percentBusUtilization", status.BusUtilization);
        Logger.recordOutput(prefix + "busOffCount", status.BusOffCount);
        Logger.recordOutput(prefix + "txFullCount", status.TxFullCount);
        Logger.recordOutput(prefix + "receiveErrorCount", status.REC);
        Logger.recordOutput(prefix + "transmitErrorCount", status.TEC);

        index++;

        if (index >= loggedBuses.size()) {
            index = 0;
        }
    }
}
