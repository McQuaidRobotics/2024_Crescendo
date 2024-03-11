package com.igknighters.util.can;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.igknighters.util.Tracer;

import monologue.MonoDashboard;

public class CANSignalManager {
    private static final String VOLT_NAME = "Voltage";
    private static final String AMP_NAME = "Current";

    private static enum SignalType {
        CONTROL, VOLT, AMP
    }

    private static final HashMap<String, HashMap<SignalType, ArrayList<BaseStatusSignal>>> signalsDatabase = new HashMap<>(64);
    private static SignalType TYPE_THIS_CYCLE = SignalType.VOLT;

    public static void registerSignals(String canbus, BaseStatusSignal... signals) {
        CANBusLogging.logBus(canbus);
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

    public static void refreshSignals() {
        for (Entry<String, HashMap<SignalType, ArrayList<BaseStatusSignal>>> entry : signalsDatabase.entrySet()) {
            Tracer.startTrace(entry.getKey());
            HashMap<SignalType, ArrayList<BaseStatusSignal>> map = entry.getValue();
            ArrayList<BaseStatusSignal> list = new ArrayList<>(64);
            if (map.containsKey(SignalType.CONTROL))
                list.addAll(map.get(SignalType.CONTROL));
            if (map.containsKey(TYPE_THIS_CYCLE))
                list.addAll(map.get(TYPE_THIS_CYCLE));
            logSignals(entry.getKey(), list);
            if (list.size() == 0)
                BaseStatusSignal.refreshAll(list.toArray(new BaseStatusSignal[list.size()]));
            Tracer.endTrace();
        }

        TYPE_THIS_CYCLE = (TYPE_THIS_CYCLE == SignalType.VOLT) ? SignalType.AMP : SignalType.VOLT;
    }

    private static void logSignals(String canbus, ArrayList<BaseStatusSignal> signals) {
        String[] array = new String[signals.size()];
        for (int i = 0; i < signals.size(); i++) {
            array[i] = signals.get(i).getName();
        }
        MonoDashboard.put("CANSignalManager/" + canbus, array);
    }
}
