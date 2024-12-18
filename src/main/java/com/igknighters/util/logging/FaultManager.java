package com.igknighters.util.logging;

import java.util.HashMap;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class FaultManager {

    private static final NetworkTable table = NetworkTableInstance
        .getDefault()
        .getTable("Faults");

    private static final HashMap<String, Integer> faultCounts = new HashMap<>();

    public static class FaultException extends RuntimeException {
        public final String name;

        public FaultException(String name, String message) {
            super(message);
            this.name = name;
        }

        public FaultException(String name, String message, Throwable cause) {
            super(message, cause);
            this.name = name;
        }

        public FaultException(String name, Throwable cause) {
            super(cause);
            this.name = name;
        }
    }

    public static void captureFault(FaultException e) {
        var sub = table.getSubTable(e.name);
        sub.getEntry("message").setString(e.getMessage());
        sub.getEntry("stacktrace").setString(e.getStackTrace().toString());
        sub.getEntry("timestamp").setDouble(Timer.getFPGATimestamp());
        sub.getEntry("cause").setString(e.getCause() == null ? "" : e.getCause().toString());
        sub.getEntry("count").setNumber(faultCounts.getOrDefault(e.name, 0) + 1);
        faultCounts.put(e.name, faultCounts.getOrDefault(e.name, 0) + 1);
    }

    public static void captureFault(String name, String message) {
        captureFault(new FaultException(name, message));
    }

    public static void captureFault(String name, String message, Throwable cause) {
        captureFault(new FaultException(name, message, cause));
    }

    public static void captureFault(String name, Throwable cause) {
        captureFault(new FaultException(name, cause));
    }

    public static void captureFault(Enum<?> name, String message) {
        captureFault(name.name(), message);
    }

    public static void captureFault(Enum<?> name, String message, Throwable cause) {
        captureFault(name.name(), message, cause);
    }

    public static void captureFault(Enum<?> name, Throwable cause) {
        captureFault(name.name(), cause);
    }

    public static void captureFault(String componentName, StatusCode code) {
        if (code == StatusCode.OK) {
            return;
        }
        captureFault(componentName, code.toString());
    }

    public static void captureFault(Enum<?> component, StatusCode code) {
        if (code == StatusCode.OK) {
            return;
        }
        captureFault(component.name(), code.toString());
    }

    public static void captureFault(Enum<?> component, BaseStatusSignal... signals) {
        for (BaseStatusSignal signal : signals) {
            StatusCode code = signal.getStatus();
            if (code == StatusCode.OK) {
                return;
            }
            captureFault(component.name() + ":" + signal.getName(), code.toString());
        }
    }
}
