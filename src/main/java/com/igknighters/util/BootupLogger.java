package com.igknighters.util;

import org.littletonrobotics.junction.Logger;

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

        System.out.println(println_prefix + message);
    }
}
