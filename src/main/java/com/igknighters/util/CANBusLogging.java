package com.igknighters.util;

import java.util.HashSet;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.igknighters.Robot;

public class CANBusLogging {
    private static final HashSet<String> loggedBuses = new HashSet<>();
    private static final String path = "CANBus";

    public static void logBus(String busName) {
        if (Robot.isSimulation() || loggedBuses.contains(busName)) {
            return;
        }
        loggedBuses.add(busName);
    }

    public static void run() {
        for (String busName : loggedBuses) {
            var status = CANBus.getStatus(busName);

            String prefix = path + "/" + busName + "/";

            Logger.recordOutput(prefix + "status", status.Status.name());
            Logger.recordOutput(prefix + "percentBusUtilization", status.BusUtilization);
            Logger.recordOutput(prefix + "busOffCount", status.BusOffCount);
            Logger.recordOutput(prefix + "txFullCount", status.TxFullCount);
            Logger.recordOutput(prefix + "receiveErrorCount", status.REC);
            Logger.recordOutput(prefix + "transmitErrorCount", status.TEC);
            Logger.recordOutput(prefix + "isFd", CANBus.isNetworkFD(busName));
        }
    }
}
