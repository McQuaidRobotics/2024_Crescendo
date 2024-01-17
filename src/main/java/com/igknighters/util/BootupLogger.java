package com.igknighters.util;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;

// import com.igknighters.constants.ConstValues;

public class BootupLogger {
    private static final String println_prefix = "[bootup] ";

    /**
     * Logs a message to the bootup log.
     * 
     * The bootup log is sent to console and to AKit logger
     * 
     * @param message The message to log
     */
    public static synchronized void BootupLog(String message) {
        Logger.recordOutput("/Bootup", message);

        if (GlobalState.isUnitTest()) {
            return;
        }

        System.out.println(println_prefix + message);
    }
}
