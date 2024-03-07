package com.igknighters.util.can;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.igknighters.util.Tracer;

public class CANSignalManager {
    private static final String VOLT_NAME = "Voltage";
    private static final String AMP_NAME = "Current";

    private static enum SignalType {
        CONTROL, VOLT, AMP
    }

    private static final Map<String, Map<SignalType, ArrayList<BaseStatusSignal>>> signalsDatabase = new HashMap<>(64);
    private static SignalType TYPE_THIS_CYCLE = SignalType.VOLT;

    public static void registerSignals(String canbus, BaseStatusSignal... signals) {
        CANBusLogging.logBus(canbus);
        Map<SignalType, ArrayList<BaseStatusSignal>> map;
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
                list = map.containsKey(SignalType.VOLT) ? map.get(SignalType.VOLT) : new ArrayList<>();
                signal.setUpdateFrequency(50);
            } else if (name.contains(AMP_NAME)) {
                list = map.containsKey(SignalType.AMP) ? map.get(SignalType.AMP) : new ArrayList<>();
                signal.setUpdateFrequency(50);
            } else {
                list = map.containsKey(SignalType.CONTROL) ? map.get(SignalType.CONTROL) : new ArrayList<>();
                signal.setUpdateFrequency(100);
            }
            list.add(signal);
        }
    }

    public static void refreshSignals() {
        for (Entry<String, Map<SignalType, ArrayList<BaseStatusSignal>>> entry : signalsDatabase.entrySet()) {
            Tracer.startTrace(entry.getKey());
            Map<SignalType, ArrayList<BaseStatusSignal>> map = entry.getValue();
            ArrayList<BaseStatusSignal> list = new ArrayList<>(64);
            list.addAll(map.get(SignalType.CONTROL));
            list.addAll(map.get(TYPE_THIS_CYCLE));
            BaseStatusSignal.refreshAll(list.toArray(new BaseStatusSignal[0]));
            Tracer.endTrace();
        }

        TYPE_THIS_CYCLE = (TYPE_THIS_CYCLE == SignalType.VOLT) ? SignalType.AMP : SignalType.VOLT;
    }
}
