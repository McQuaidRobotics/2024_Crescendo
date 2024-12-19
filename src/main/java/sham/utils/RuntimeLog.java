package sham.utils;

import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.NTDataLogger;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class RuntimeLog {
    private static final StringPublisher entry;
    private static final DataLogger diagnostics;

    static {
        // we need to make sure we never log network tables through the implicit wpilib logger
        entry = NetworkTableInstance.getDefault().getStringTopic("/MapleSim/Runtime").publish();
        debug("MapleSim Runtime Logger Initialized");
        diagnostics = new NTDataLogger(NetworkTableInstance.getDefault()).getSubLogger("/MapleSim");
        debug("MapleSim Diagnostics Logger Initialized");
    }

    public static void debug(String debug) {
        entry.set("[MAPLESIM] (DEBUG) " + debug);
    }

    public static void info(String info) {
        entry.set("[MAPLESIM] (INFO) " + info);
        System.out.println("[MAPLESIM] " + info);
    }

    public static void warn(String warning) {
        entry.set("[MAPLESIM] (WARNING) " + warning);
        DriverStationJNI.sendError(false, 1, false, "[MAPLESIM] " + warning, "", "", true);
    }

    public static void error(String error) {
        entry.set("[MAPLESIM] (ERROR) " + error);
        DriverStationJNI.sendError(true, 1, false, "[MAPLESIM] " + error, "", "", true);
    }

    public static DataLogger loggerFor(String subPath) {
        return diagnostics.getSubLogger(subPath);
    }
}