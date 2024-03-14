package com.igknighters.util;

import java.util.HashMap;

import com.igknighters.constants.ConstValues;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableValues {
    private static final String pathPrefix = "Tunables/";

    private static final HashMap<String, TunableDouble> doubleTunables = new HashMap<>();
    private static final HashMap<String, TunableInteger> intTunables = new HashMap<>();
    private static final HashMap<String, TunableBoolean> boolTunables = new HashMap<>();

    public static class TunableDouble implements Cloneable {
        private final DoubleEntry entry;
        private final double defaultValue;

        public TunableDouble(String path, double defaultValue) {
            this.entry = NetworkTableInstance
                .getDefault()
                .getDoubleTopic(pathPrefix + path)
                .getEntry(defaultValue);
            entry.setDefault(defaultValue);
            this.defaultValue = defaultValue;
        }

        private TunableDouble(DoubleEntry entry, double defaultValue) {
            this.entry = entry;
            this.defaultValue = defaultValue;
        }

        public double get() {
            return ConstValues.DEBUG ? entry.get(defaultValue) : defaultValue;
        }

        @Override
        public TunableDouble clone() {
            return new TunableDouble(entry, defaultValue);
        }
    }

    public static TunableDouble getDouble(String path, double defaultValue) {
        if (doubleTunables.containsKey(path)) {
            return doubleTunables.get(path);
        } else {
            TunableDouble newTunable = new TunableDouble(path, defaultValue);
            doubleTunables.put(path, newTunable);
            return newTunable;
        }
    }

    public static class TunableInteger implements Cloneable {
        private final IntegerEntry entry;
        private final int defaultValue;

        public TunableInteger(String path, int defaultValue) {
            this.entry = NetworkTableInstance
                .getDefault()
                .getIntegerTopic(pathPrefix + path)
                .getEntry(defaultValue);
            entry.setDefault(defaultValue);
            this.defaultValue = defaultValue;
        }

        private TunableInteger(IntegerEntry entry, int defaultValue) {
            this.entry = entry;
            this.defaultValue = defaultValue;
        }

        public int get() {
            return (int) (ConstValues.DEBUG ? entry.get(defaultValue) : defaultValue);
        }

        @Override
        public TunableInteger clone() {
            return new TunableInteger(entry, defaultValue);
        }
    }

    public static TunableInteger getInteger(String path, int defaultValue) {
        if (intTunables.containsKey(path)) {
            return intTunables.get(path);
        } else {
            TunableInteger newTunable = new TunableInteger(path, defaultValue);
            intTunables.put(path, newTunable);
            return newTunable;
        }
    }

    public static class TunableBoolean implements Cloneable {
        private final BooleanEntry entry;
        private final boolean defaultValue;

        public TunableBoolean(String path, boolean defaultValue) {
            this.entry = NetworkTableInstance
                .getDefault()
                .getBooleanTopic(pathPrefix + path)
                .getEntry(defaultValue);
            entry.setDefault(defaultValue);
            this.defaultValue = defaultValue;
        }

        private TunableBoolean(BooleanEntry entry, boolean defaultValue) {
            this.entry = entry;
            this.defaultValue = defaultValue;
        }

        public boolean get() {
            return ConstValues.DEBUG ? entry.get(defaultValue) : defaultValue;
        }

        @Override
        public TunableBoolean clone() {
            return new TunableBoolean(entry, defaultValue);
        }
    }

    public static TunableBoolean getBoolean(String path, boolean defaultValue) {
        if (boolTunables.containsKey(path)) {
            return boolTunables.get(path);
        } else {
            TunableBoolean newTunable = new TunableBoolean(path, defaultValue);
            boolTunables.put(path, newTunable);
            return newTunable;
        }
    }
}
