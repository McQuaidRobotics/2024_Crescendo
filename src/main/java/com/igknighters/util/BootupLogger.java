package com.igknighters.util;

import org.littletonrobotics.junction.Logger;

// import com.igknighters.constants.ConstValues;

public class BootupLogger {
    private static final String println_prefix = "[bootup] ";
    public static synchronized void BootupLog(String message) {
        Logger.recordOutput("/Bootup", message);

        System.out.println(println_prefix + message);
    }
}
