package sham.utils.mathutils;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.util.struct.Struct;
import monologue.procstruct.ProceduralStructGenerator;

public class MeasureMath {
    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M abs(M m) {
        return (M) m.unit().ofBaseUnits(Math.abs(m.baseUnitMagnitude()));
    }

    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M negate(M m) {
        return (M) m.unit().ofBaseUnits(-m.baseUnitMagnitude());
    }

    public static <U extends Unit, M extends Measure<U>> M max(M m1, M m2) {
        return m1.baseUnitMagnitude() > m2.baseUnitMagnitude() ? m1 : m2;
    }

    public static <U extends Unit, M extends Measure<U>> M min(M m1, M m2) {
        return m1.baseUnitMagnitude() < m2.baseUnitMagnitude() ? m1 : m2;
    }

    public static <U extends Unit,M extends Measure<U>> M clamp(M m, M min, M max) {
        return max(min, min(max, m));
    }

    public static <U extends Unit, M extends Measure<U>> M clamp(M m, M magnitudeLimit) {
        return max(negate(abs(magnitudeLimit)), min(abs(magnitudeLimit), m));
    }

    public static <U extends Unit, M extends Measure<U>> double signum(M m) {
        return Math.signum(m.baseUnitMagnitude());
    }

    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M nudgeZero(M m, M tolerance) {
        return abs(m).baseUnitMagnitude() < tolerance.baseUnitMagnitude()
            ? (M) m.unit().zero()
            : m;
    }

    public record XY<M extends Measure<?>>(M x, M y) {
        public static XY<Distance> of(Translation2d t) {
            return new XY<>(Meters.of(t.getX()), Meters.of(t.getY()));
        }

        @SuppressWarnings("rawtypes")
        public static final Struct<XY> struct = ProceduralStructGenerator.genRecord(XY.class);
    }

    public static Torque times(Distance d, Force f) {
        return NewtonMeters.of(d.in(Meters) * f.in(Newtons));
    }

    public static AngularAcceleration div(Torque t, MomentOfInertia moi) {
        return RadiansPerSecondPerSecond.of(t.in(NewtonMeters) / moi.in(KilogramSquareMeters));
    }

    public static Force times(LinearAcceleration a, Mass m) {
        return Newtons.of(m.in(Kilograms) * a.in(MetersPerSecondPerSecond));
    }

    public static Torque times(AngularAcceleration a, MomentOfInertia moi) {
        return NewtonMeters.of(moi.in(KilogramSquareMeters) * a.in(RadiansPerSecondPerSecond));
    }
}
