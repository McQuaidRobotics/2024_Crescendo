package com.igknighters.util.can;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.BaseStatusSignal;
import com.igknighters.util.logging.Tracer;

/**
 * A utility to clump CAN signals together and update them all at once.
 * This has shown a performance improvement over updating each signal
 * individually and updating all signals per device at once.
 */
public class CANSignalManager {
    private static final String VOLT_NAME = "Voltage";
    private static final String AMP_NAME = "Current";

    private static enum SignalType {
        CONTROL, VOLT, AMP
    }

    private static final AtomicBoolean CHARACTERIZATION_MODE = new AtomicBoolean(false);

    private static final HashMap<String, HashMap<SignalType, ArrayList<BaseStatusSignal>>> signalsDatabase = new HashMap<>(64);
    private static SignalType TYPE_THIS_CYCLE = SignalType.VOLT;

    /**
     * Registers a list of signals to be updated in the CANSignalManager
     * 
     * @param canbus The name of the CAN bus to register the signals to
     * @param signals The signals to register
     */
    public static void registerSignals(String canbus, BaseStatusSignal... signals) {
        HashMap<SignalType, ArrayList<BaseStatusSignal>> map;
        if (signalsDatabase.containsKey(canbus)) {
            map = signalsDatabase.get(canbus);
        } else {
            map = new HashMap<>(16);
            signalsDatabase.put(canbus, map);
        }
        for (BaseStatusSignal signal : signals) {
            String name = signal.getName();
            List<BaseStatusSignal> list;
            if (name.contains(VOLT_NAME)) {
                if (!map.containsKey(SignalType.VOLT)) map.put(SignalType.VOLT, new ArrayList<>());
                list = map.get(SignalType.VOLT);
                signal.setUpdateFrequency(50);
            } else if (name.contains(AMP_NAME)) {
                if (!map.containsKey(SignalType.AMP)) map.put(SignalType.AMP, new ArrayList<>());
                list = map.get(SignalType.AMP);
                signal.setUpdateFrequency(50);
            } else {
                if (!map.containsKey(SignalType.CONTROL)) map.put(SignalType.CONTROL, new ArrayList<>());
                list = map.get(SignalType.CONTROL);
                signal.setUpdateFrequency(100);
            }
            list.add(signal);
        }
    }

    /**
     * Refreshes all signals in the CANSignalManager, should be called once per cycle
     */
    public static void refreshSignals() {
        for (Entry<String, HashMap<SignalType, ArrayList<BaseStatusSignal>>> entry : signalsDatabase.entrySet()) {
            Tracer.startTrace(entry.getKey());
            HashMap<SignalType, ArrayList<BaseStatusSignal>> map = entry.getValue();
            ArrayList<BaseStatusSignal> list = new ArrayList<>(64);
            if (map.containsKey(SignalType.CONTROL))
                list.addAll(map.get(SignalType.CONTROL));
            if (CHARACTERIZATION_MODE.get()) {
                list.addAll(map.get(SignalType.VOLT));
                list.addAll(map.get(SignalType.AMP));
            } else {
                if (map.containsKey(TYPE_THIS_CYCLE))
                    list.addAll(map.get(TYPE_THIS_CYCLE));
            }
            if (list.size() != 0)
                BaseStatusSignal.refreshAll(list.toArray(new BaseStatusSignal[list.size()]));
            Tracer.endTrace();
        }

        TYPE_THIS_CYCLE = (TYPE_THIS_CYCLE == SignalType.VOLT) ? SignalType.AMP : SignalType.VOLT;
    }

    /**
     * Changes the behavior to be more accurate for mechanism characterization
     * 
     * @param mode True to enable characterization mode, false to disable
     */
    public static void setCharacterizationMode(boolean mode) {
        CHARACTERIZATION_MODE.set(mode);

        //update all volt and amp signals update frequency
        final int freq = mode ? 50 : 250;

        for (Entry<String, HashMap<SignalType, ArrayList<BaseStatusSignal>>> entry : signalsDatabase.entrySet()) {
            ArrayList<BaseStatusSignal> list = new ArrayList<>(128);
            HashMap<SignalType, ArrayList<BaseStatusSignal>> map = entry.getValue();
            if (map.containsKey(SignalType.VOLT))
                list.addAll(map.get(SignalType.VOLT));
            if (map.containsKey(SignalType.AMP))
                list.addAll(map.get(SignalType.AMP));

            BaseStatusSignal.setUpdateFrequencyForAll(
                freq,
                list.toArray(new BaseStatusSignal[list.size()])
            );
        }
    }
}
