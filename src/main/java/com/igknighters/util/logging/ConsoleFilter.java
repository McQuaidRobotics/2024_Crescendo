package com.igknighters.util.logging;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.function.Function;

import com.igknighters.Robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Allows using regex to filter console output and send it to other sources or ignore it altogether
 */
public class ConsoleFilter {
    public static interface Diverter {
        public void divert(String s);
    }

    public static class NullDiverter implements Diverter {
        @Override
        public void divert(String s) {
            // Do nothing
        }
    }

    public static class DatalogDiverter implements Diverter {
        private final StringLogEntry entry;

        public DatalogDiverter(String key) {
            entry = new StringLogEntry(DataLogManager.getLog(), key);
        }

        @Override
        public void divert(String s) {
            entry.append(s);
        }
    }

    public static class NetworkTablesDiverter implements Diverter {
        private final StringEntry entry;

        public NetworkTablesDiverter(String key) {
            entry = NetworkTableInstance.getDefault().getStringTopic(key).getEntry("");
        }

        @Override
        public void divert(String s) {
            entry.set(s);
        }
    }

    public static interface Filter {
        public boolean filter(String s);
    }

    public static class RegexFilter implements Filter {
        private final String regex;

        public RegexFilter(String regex) {
            this.regex = regex;
        }

        @Override
        public boolean filter(String s) {
            return s.matches(regex);
        }
    }

    public static class CustomFilter implements Filter {
        private final Function<String, Boolean> filter;

        public CustomFilter(Function<String, Boolean> filter) {
            this.filter = filter;
        }

        @Override
        public boolean filter(String s) {
            return filter.apply(s);
        }
    }

    public static record FilteredDiverter(Filter filter, Diverter diverter) {}

    private static final PrintStream originalOut;

    private static final PrintStream newOut;

    private static final ArrayList<FilteredDiverter> diverters = new ArrayList<>();

    private static boolean filterAndDiver(String s) {
        for (FilteredDiverter fd : diverters) {
            if (fd.filter.filter(s)) {
                fd.diverter.divert(s);
                return true;
            }
        }
        return false;
    }

    static {
        if (Robot.isUnitTest()) {
            originalOut = null;

            newOut = null;
        } else {
            originalOut = System.out;

            newOut = new PrintStream(originalOut) {
                @Override
                public void print(String s) {
                    if (!filterAndDiver(s)) originalOut.print(s);
                }

                @Override
                public void println(String x) {
                    if (!filterAndDiver(x)) originalOut.println(x);
                }
            };

            System.setOut(newOut);
        }
    }

    public static void addFilter(Filter filter, Diverter diverter) {
        diverters.add(new FilteredDiverter(filter, diverter));
    }
}
