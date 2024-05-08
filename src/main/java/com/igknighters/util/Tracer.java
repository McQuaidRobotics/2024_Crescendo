package com.igknighters.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A Utility class for tracing code execution time.
 * Will put info to NetworkTables under the "Tracer" table.
 * 
 * <pre><code>
 * 
 * @Override
 * public void loopFunc() {
 *    Tracer.traceFunc("LoopFunc", super::loopFunc);
 * }
 * 
 * <p>
 * 
 * @Override
 * public void robotPeriodic() {
 *     Tracer.startTrace("RobotPeriodic");
 *     Tracer.traceFunc("CommandScheduler", scheduler::run);
 *     Tracer.traceFunc("Monologue", Monologue::updateAll);
 *     Tracer.endTrace();
 * }
 * 
 * </code></pre>
 */
public class Tracer {
    private static final ArrayList<String> trace = new ArrayList<>();
    private static final HashMap<String, Double> traceTimes = new HashMap<>();
    private static final HashMap<String, Double> traceStartTimes = new HashMap<>();
    private static final NetworkTable rootTable = NetworkTableInstance.getDefault().getTable("Tracer");
    private static final HashMap<String, NetworkTableEntry> entryHeap = new HashMap<>();

    private static boolean threadValidation = false;
    private static long tracedThread = 0;

    private static String traceStack() {
        StringBuilder sb = new StringBuilder();
        for (String s : trace) {
            sb.append(s);
            sb.append("/");
        }
        return sb.toString().substring(0, sb.length() - 1);
    }

    /**
     * Enables thread validation on {@link Tracer#startTrace(String)}
     * and functions that use that like {@link Tracer#traceFunc(String, Runnable)}.
     * 
     * Thread validation will check if the thread that called the first {@link Tracer#startTrace(String)}
     * is the same thread that calls are further {@link Tracer#startTrace(String)}.
     * 
     * If not the same thread, a {@link DriverStation#reportError(String, boolean)} will be called with the message.
     * 
     * This is not on by default due to the performance overhead this introduces.
     */
    public static void enableThreadValidation() {
        threadValidation = true;
    }

    /**
     * Starts a trace,
     * should be called at the beginning of a function thats not being called by
     * user code.
     * Should be paired with {@link Tracer#endTrace()} at the end of the function.
     * 
     * Best used in periodic functions in Subsystems and Robot.java.
     * 
     * @param name the name of the trace, should be unique to the function.
     */
    public static void startTrace(String name) {
        if (threadValidation) {
            if (tracedThread == 0) {
                tracedThread = Thread.currentThread().getId();
            } else if (tracedThread != Thread.currentThread().getId()) {
                DriverStation.reportError("[Tracer] Tracer is being used by multiple threads", true);
            }
        }
        trace.add(name);
        traceStartTimes.put(traceStack(), Timer.getFPGATimestamp() * 1_000.0);
    }

    /**
     * Ends a trace, should only be called at the end of a function thats not being
     * called by user code.
     * If a {@link Tracer#startTrace(String)} is not paired with a
     * {@link Tracer#endTrace()} there could be a crash.
     */
    public static void endTrace() {
        try {
            var startTime = traceStartTimes.get(traceStack());
            traceTimes.put(traceStack(), Timer.getFPGATimestamp() * 1_000.0 - startTime);
            trace.remove(trace.size() - 1);
            if (trace.size() == 0) {
                endCycle();
            }
        } catch (Exception e) {
            DriverStation.reportError("[Tracer] An end trace was called with no opening trace " + e, true);
        }
    }

    /**
     * Traces a function, should be used in place of
     * {@link Tracer#startTrace(String)} and {@link Tracer#endTrace()}
     * for functions called by user code like {@code CommandScheduler.run()} and
     * other expensive functions.
     * 
     * @param name     the name of the trace, should be unique to the function.
     * @param runnable the function to trace.
     * 
     * @apiNote If you want to return a value then use
     *          {@link Tracer#traceFunc(String, Supplier)}.
     */
    public static void traceFunc(String name, Runnable runnable) {
        startTrace(name);
        runnable.run();
        endTrace();
    }

    /**
     * Traces a function, should be used in place of
     * {@link Tracer#startTrace(String)} and {@link Tracer#endTrace()}
     * for functions called by user code like {@code CommandScheduler.run()} and
     * other expensive functions.
     * 
     * @param name     the name of the trace, should be unique to the function.
     * @param supplier the function to trace.
     */
    public static <T> T traceFunc(String name, Supplier<T> supplier) {
        startTrace(name);
        T ret = supplier.get();
        endTrace();
        return ret;
    }

    private static void endCycle() {
        var keys = new ArrayList<String>();
        entryHeap
                .entrySet()
                .stream()
                .filter(mapEntry -> !traceTimes.containsKey(mapEntry.getKey()))
                .forEach(mapEntry -> {
                    keys.add(mapEntry.getKey());
                    mapEntry.getValue().unpublish();
                });
        keys.forEach(key -> entryHeap.remove(key));

        for (var trace : traceTimes.entrySet()) {
            NetworkTableEntry entry;
            if (!entryHeap.containsKey(trace.getKey())) {
                entry = rootTable.getEntry(trace.getKey());
                entryHeap.put(trace.getKey(), entry);
            } else {
                entry = entryHeap.get(trace.getKey());
            }
            entry.setDouble(trace.getValue());
        }
        traceTimes.clear();
    }
}
