package com.igknighters.util.can;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public class CANRetrier {

    public static class CANRetryError extends RuntimeException {

        public CANRetryError() {
            super("Can retry limit exceded, marked fatal!!!!");
        }
    } 

    /**
     * Given a CTRE Status Code will retry a given amount of times and return true if the code ever returns ok
     * 
     * @param statusCodeSup The status code.
     * @param retryLimit The retry limit.
     * @return Whether the status code is ok.
     */
    public static boolean retryStatusCode(Supplier<StatusCode> statusCodeSup, int retryLimit) {
        for (int i = 0; i < retryLimit; i++) {
            if (statusCodeSup.get().isOK()) return true;
        }
        DriverStation.reportWarning("Status Code " + statusCodeSup.get().getName() + " is NOT ok!", true);
        return false;
    }

    /**
     * Given a CTRE Status Code will retry a given amount of times and return true if the code ever returns ok, else will crash the code with an error.
     * 
     * @param statusCodeSup The status code.
     * @param retryLimit The retry limit.
     * @return Whether the status code is ok.
     */
    public static boolean retryStatusCodeFatal(Supplier<StatusCode> statusCodeSup, int retryLimit) {
        for (int i = 0; i < retryLimit; i++) {
            if (statusCodeSup.get().isOK()) return true;
        }
        throw new CANRetryError();
    }
}
