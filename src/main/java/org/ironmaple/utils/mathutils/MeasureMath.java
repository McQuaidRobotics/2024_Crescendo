package org.ironmaple.utils.mathutils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class MeasureMath {
    @SuppressWarnings("unchecked")
    public static <U extends Unit> Measure<U> abs(Measure<U> m) {
        Unit unit = m.unit();
        return (Measure<U>) unit.ofBaseUnits(Math.abs(m.baseUnitMagnitude()));
    }

    public static <M extends Measure<?>> M max(M m1, M m2) {
        return m1.baseUnitMagnitude() > m2.baseUnitMagnitude() ? m1 : m2;
    }

    public static <M extends Measure<?>> M min(M m1, M m2) {
        return m1.baseUnitMagnitude() < m2.baseUnitMagnitude() ? m1 : m2;
    }

    public static <M extends Measure<?>> double signum(M m) {
        return Math.signum(m.baseUnitMagnitude());
    }
}
