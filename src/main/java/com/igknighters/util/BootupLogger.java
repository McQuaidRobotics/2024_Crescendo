package com.igknighters.util;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

// import com.igknighters.constants.ConstValues;

public class BootupLogger {
    private static final StringLogEntry entry = new StringLogEntry(DataLogManager.getLog(), "/Bootup");
    private static final String println_prefix = "[bootup] ";
    static {
        entry.append("Bootup Logger initialized");
    }
    public static synchronized void BootupLog(String message) {
        entry.append(message);
        // if (ConstValues.DEBUG) {
            System.out.println(println_prefix + message);
        // }
    }
}
