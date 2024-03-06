package com.igknighters.util;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.igknighters.Robot;

import edu.wpi.first.wpilibj.Timer;
import monologue.MonologueDashboard;

public class CANBusLogging {
    private static final ArrayList<String> loggedBuses = new ArrayList<>();
    private static final String path = "CANBus";
    private static final Timer timer = new Timer();
    private static int index = 0;

    public static void logBus(String busName) {
        if (Robot.isSimulation() || loggedBuses.contains(busName)) {
            return;
        }

        MonologueDashboard.put(
                path + "/" + busName + "/" + "isFd",
                CANBus.isNetworkFD(busName));

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

        MonologueDashboard.put(prefix + "status", status.Status.name());
        MonologueDashboard.put(prefix + "percentBusUtilization", status.BusUtilization);
        MonologueDashboard.put(prefix + "busOffCount", status.BusOffCount);
        MonologueDashboard.put(prefix + "txFullCount", status.TxFullCount);
        MonologueDashboard.put(prefix + "receiveErrorCount", status.REC);
        MonologueDashboard.put(prefix + "transmitErrorCount", status.TEC);

        index++;

        if (index >= loggedBuses.size()) {
            index = 0;
        }
    }
}
