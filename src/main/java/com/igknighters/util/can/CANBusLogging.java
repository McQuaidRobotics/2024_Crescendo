package com.igknighters.util.can;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.igknighters.Robot;

import edu.wpi.first.wpilibj.Timer;
import monologue.MonoDashboard;

/**
 * A utility to log CAN bus status to the dashboard.
 */
public class CANBusLogging {
    private static final ArrayList<String> loggedBuses = new ArrayList<>();
    private static final String path = "CANBus";
    private static final Timer timer = new Timer();
    private static int index = 0;

    /**
     * Logs a CAN bus in telemetry.
     * 
     * @param busName The name of the bus to log
     */
    public static void logBus(String busName) {
        if (Robot.isSimulation() || loggedBuses.contains(busName)) {
            return;
        }

        MonoDashboard.put(
                path + "/" + busName + "/" + "isFd",
                CANBus.isNetworkFD(busName));

        timer.start();

        loggedBuses.add(busName);
    }

    /**
     * Runs the CAN bus logging.
     */
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

        MonoDashboard.put(prefix + "status", status.Status.name());
        MonoDashboard.put(prefix + "percentBusUtilization", status.BusUtilization);
        MonoDashboard.put(prefix + "busOffCount", status.BusOffCount);
        MonoDashboard.put(prefix + "txFullCount", status.TxFullCount);
        MonoDashboard.put(prefix + "receiveErrorCount", status.REC);
        MonoDashboard.put(prefix + "transmitErrorCount", status.TEC);

        index++;

        if (index >= loggedBuses.size()) {
            index = 0;
        }
    }
}
