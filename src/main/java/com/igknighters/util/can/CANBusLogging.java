package com.igknighters.util.can;

import java.util.ArrayList;

import com.ctre.phoenix6.CANBus;
import com.igknighters.Robot;

import edu.wpi.first.wpilibj.Timer;
import monologue.Monologue;

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

        Monologue.log(
                path + "/" + busName + "/" + "isFd",
                CANBus.isNetworkFD(busName));

        timer.start();

        loggedBuses.add(busName);
    }

    /**
     * Runs the CAN bus logging.
     */
    public static void log() {
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

        Monologue.log(prefix + "status", status.Status.name());
        Monologue.log(prefix + "percentBusUtilization", status.BusUtilization);
        Monologue.log(prefix + "busOffCount", status.BusOffCount);
        Monologue.log(prefix + "txFullCount", status.TxFullCount);
        Monologue.log(prefix + "receiveErrorCount", status.REC);
        Monologue.log(prefix + "transmitErrorCount", status.TEC);

        index++;

        if (index >= loggedBuses.size()) {
            index = 0;
        }
    }
}
