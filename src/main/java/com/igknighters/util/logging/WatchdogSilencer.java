package com.igknighters.util.logging;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.Watchdog;

/**
 * A crude utility to make private watchdogs shut up.
 */
public class WatchdogSilencer {

    private static void silenceWatchdog(Object obj) {
        Watchdog watchdog = (Watchdog) obj;
        watchdog.disable();
        watchdog.setTimeout(100000.0);
    }

    public static Field getField(Object obj, String member) {
        ArrayList<Field> result = new ArrayList<Field>();

        Class<?> i = obj.getClass();
        while (i != null && i != Object.class) {
            Collections.addAll(result, i.getDeclaredFields());
            i = i.getSuperclass();
        }

        for (Field field : result) {
            if (field.getName().equals(member)) {
                return field;
            }
        }

        throw new RuntimeException("Field not found: " + member);
    }

    /**
     * Silences the watchdog in the given object under the given member name.
     * 
     * @param obj The object containing the watchdog
     * @param member The name of the member containing the watchdog
     */
    public static void silence(Object obj, String member) {
        try {
            Field field = getField(obj, member);
            field.setAccessible(true);
            silenceWatchdog(field.get(obj));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
