package com.igknighters.util.logging;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.Consumer;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
 *     Tracer.traceFunc("MyVendorDep", MyVendorDep::updateAll);
 *     Tracer.endTrace();
 * }
 * 
 * </code></pre>
 */
public class Tracer {
    private static final class TraceStartData {
        private double startTime = 0.0;
        private double startGCTotalTime = 0.0;

        private void set(double startTime, double startGCTotalTime) {
            this.startTime = startTime;
            this.startGCTotalTime = startGCTotalTime;
        }
    }
    private static final class TraceDurationLogger {
        private Optional<DoublePublisher> networkLogger;
        private Optional<DoubleLogEntry> localLogger;
    }

    /**
     * All of the tracers persistent state in a single object to be stored in a
     * {@link ThreadLocal}.
     */
    private static final class TracerState {
        private final NetworkTable rootTable;

        // the stack of traces, every startTrace will add to this stack
        // and every endTrace will remove from this stack
        private final ArrayList<String> traceStack = new ArrayList<>();
        // ideally we only need `traceStack` but in the interest of memory optimization
        // and string concatenation speed we store the history of the stack to reuse the stack names
        private final ArrayList<String> traceStackHistory = new ArrayList<>();
        // the time of each trace, the key is the trace name, modified every endTrace
        private final HashMap<String, Double> traceTimes = new HashMap<>();
        // the start time of each trace and the gc time at the start of the trace,
        // the key is the trace name, modified every startTrace and endTrace.
        private final HashMap<String, TraceStartData> traceStartTimes = new HashMap<>();
        // the publishers for each trace, the key is the trace name, modified every endCycle
        private final HashMap<String, DoublePublisher> publisherHeap = new HashMap<>();

        // the garbage collector beans
        private final ArrayList<GarbageCollectorMXBean> gcs = new ArrayList<>(
                ManagementFactory.getGarbageCollectorMXBeans());
        private final DoublePublisher gcTimeEntry;
        private double gcTimeThisCycle = 0.0;

        private TracerState(String threadName) {
            if (threadName == null) {
                this.rootTable = NetworkTableInstance.getDefault().getTable("Tracer");
            } else {
                this.rootTable = NetworkTableInstance.getDefault().getTable("Tracer").getSubTable(threadName);
            }
            this.gcTimeEntry = rootTable.getDoubleTopic("GCTime").publishEx(
                    "double",
                    "{ \"cached\": false}");
        }

        private String appendTraceStack(String trace) {
            traceStack.add(trace);
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < traceStack.size(); i++) {
                sb.append(traceStack.get(i));
                if (i < traceStack.size() - 1) {
                    sb.append("/");
                }
            }
            String str = sb.toString();
            traceStackHistory.add(str);
            return str;
        }

        private String popTraceStack() {
            traceStack.remove(traceStack.size() - 1);
            return traceStackHistory.remove(traceStackHistory.size() - 1);
        }

        private double totalGCTime() {
            double gcTime = 0;
            for (GarbageCollectorMXBean gc : gcs) {
                gcTime += gc.getCollectionTime();
            }
            return gcTime;
        }

        private void endCycle() {
            // update times for all already existing publishers
            for (var publisher : publisherHeap.entrySet()) {
                Double time = traceTimes.remove(publisher.getKey());
                if (time == null)
                    time = 0.0;
                publisher.getValue().set(time);
            }
            // create publishers for all new entries
            for (var traceTime : traceTimes.entrySet()) {
                DoublePublisher publisher = rootTable.getDoubleTopic(traceTime.getKey()).publishEx(
                        "double",
                        "{ \"cached\": false}");
                publisher.set(traceTime.getValue());
                publisherHeap.put(traceTime.getKey(), publisher);
            }

            // log gc time
            if (gcs.size() > 0)
                gcTimeEntry.set(gcTimeThisCycle);
            gcTimeThisCycle = 0.0;

            // clean up state
            traceTimes.clear();
            traceStackHistory.clear();
        }
    }

    private static final AtomicBoolean singleThreadedMode = new AtomicBoolean(false);
    private static final AtomicBoolean anyTracesStarted = new AtomicBoolean(false);
    private static final ThreadLocal<TracerState> threadLocalState = ThreadLocal.withInitial(() -> {
        if (singleThreadedMode.get()) {
            throw new IllegalStateException("Single threaded mode is enabled, cannot create new TracerState");
        }
        anyTracesStarted.set(true);
        return new TracerState(Thread.currentThread().getName());
    });

    private static void startTraceInner(final String name, final TracerState state) {
        String stack = state.appendTraceStack(name);
        TraceStartData data = state.traceStartTimes.get(stack);
        if (data == null) {
            data = new TraceStartData();
            state.traceStartTimes.put(stack, data);
        }
        data.set(Timer.getFPGATimestamp() * 1_000.0, state.totalGCTime());
    }

    private static void endTraceInner(final TracerState state) {
        try {
            String stack = state.popTraceStack();
            var startData = state.traceStartTimes.get(stack);
            double gcTimeSinceStart = state.totalGCTime() - startData.startGCTotalTime;
            state.gcTimeThisCycle += gcTimeSinceStart;
            state.traceTimes.put(
                    stack,
                    Timer.getFPGATimestamp() * 1_000.0
                            - startData.startTime
                            - gcTimeSinceStart);
            if (state.traceStack.size() == 0) {
                state.endCycle();
            }
        } catch (Exception e) {
            DriverStation.reportError("[Tracer] An end trace was called with no opening trace " + e, true);
            for (var trace : state.traceStack) {
                DriverStation.reportError("[Tracer] Open trace: " + trace, false);
            }
            state.endCycle();
        }
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
        startTraceInner(name, threadLocalState.get());
    }

    /**
     * Ends a trace, should only be called at the end of a function thats not being
     * called by user code.
     * If a {@link Tracer#startTrace(String)} is not paired with a
     * {@link Tracer#endTrace()} there could be a crash.
     */
    public static void endTrace() {
        endTraceInner(threadLocalState.get());
    }

    /**
     * Disables garbage collection logging for the current thread.
     * This can help performance in some cases.
     * 
     * <p>This counts as starting a tracer on the current thread,
     * this is important to consider with {@link Tracer#enableSingleThreadedMode()}
     * and should never be called before if you are using single threaded mode.
     */
    public static void disableGcLoggingForCurrentThread() {
        TracerState state = threadLocalState.get();
        state.gcTimeEntry.close();
        state.gcs.clear();
    }

    /**
     * Enables single threaded mode for the Tracer.
     * This will cause traces on different threads to throw an exception.
     * This will shorten the path of traced data in NetworkTables by not including the thread name.
     * 
     * <p><b>Warning:</b> This will throw an exception if called after any traces have been started.
     */
    public static void enableSingleThreadedMode() {
        if (anyTracesStarted.get()) {
            throw new IllegalStateException("Cannot enable single threaded mode after traces have been started");
        }
        threadLocalState.set(new TracerState(null));
        singleThreadedMode.set(true);
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
        final TracerState state = threadLocalState.get();
        startTraceInner(name, state);
        runnable.run();
        endTraceInner(state);
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
    public static <R> R traceFunc(String name, Supplier<R> supplier) {
        final TracerState state = threadLocalState.get();
        startTraceInner(name, state);
        R ret = supplier.get();
        endTraceInner(state);
        return ret;
    }

    // A REIMPLEMENTATION OF THE OLD TRACER TO NOT BREAK OLD CODE

    private static final long kMinPrintPeriod = 1000000; // microseconds

    private long m_lastEpochsPrintTime; // microseconds
    private long m_startTime; // microseconds

    private final HashMap<String, Long> m_epochs = new HashMap<>(); // microseconds

    /** 
     * A {@code Tracer} constructor compatible with the 2024 {@code Tracer}.
     * 
     * @deprecated This constructor is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
    */
    @Deprecated(since = "2025", forRemoval = true)
    public Tracer() {
        resetTimer();
    }

    /** Clears all epochs.
     * 
     * @deprecated This method is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
    */
    @Deprecated(since = "2025", forRemoval = true)
    public void clearEpochs() {
        m_epochs.clear();
        resetTimer();
    }

    /** Restarts the epoch timer.
     * 
     * @deprecated This method is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
    */
    @Deprecated(since = "2025", forRemoval = true)
    public final void resetTimer() {
        m_startTime = RobotController.getFPGATime();
    }

    /**
     * Adds time since last epoch to the list printed by printEpochs().
     *
     * <p>Epochs are a way to partition the time elapsed so that when overruns occur, one can
     * determine which parts of an operation consumed the most time.
     *
     * <p>This should be called immediately after execution has finished, with a call to this method
     * or {@link #resetTimer()} before execution.
     *
     * @param epochName The name to associate with the epoch.
     * 
     * @deprecated This method is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
     */
    @Deprecated(since = "2025", forRemoval = true)
    public void addEpoch(String epochName) {
        long currentTime = RobotController.getFPGATime();
        m_epochs.put(epochName, currentTime - m_startTime);
        m_startTime = currentTime;
    }

    /**
     * Prints list of epochs added so far and their times to the DriverStation.
     * 
     * @deprecated This method is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
     */
    @Deprecated(since = "2025", forRemoval = true)
    public void printEpochs() {
        printEpochs(out -> DriverStation.reportWarning(out, false));
    }

    /**
     * Prints list of epochs added so far and their times to the entered String consumer.
     *
     * <p>This overload can be useful for logging to a file, etc.
     *
     * @param output the stream that the output is sent to
     * 
     * @deprecated This method is only for compatibility with the 2024 {@code Tracer} and will be removed in 2025.
     * Use the static methods in {@link Tracer} instead.
     */
    @Deprecated(since = "2025", forRemoval = true)
    public void printEpochs(Consumer<String> output) {
        long now = RobotController.getFPGATime();
        if (now - m_lastEpochsPrintTime > kMinPrintPeriod) {
            StringBuilder sb = new StringBuilder();
            m_lastEpochsPrintTime = now;
            m_epochs.forEach(
                    (key, value) -> sb.append(String.format("\t%s: %.6fs\n", key, value / 1.0e6)));
            if (sb.length() > 0) {
                output.accept(sb.toString());
            }
        }
    }
}
