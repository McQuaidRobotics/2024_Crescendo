package com.igknighters.util;

import com.igknighters.Robot;

import monologue.MonoDashboard;

public class BootupLogger {
    private static final String println_prefix = "[Bootup] ";

    /**
     * Logs a message to the bootup log.
     * 
     * The bootup log is sent to console and to AKit logger
     * 
     * @param message The message to log
     */
    public static synchronized void bootupLog(String message) {
        MonoDashboard.put("/Bootup", message);

        if (!Robot.isUnitTest()) {
            System.out.println(println_prefix + message);
        }
    }
}
