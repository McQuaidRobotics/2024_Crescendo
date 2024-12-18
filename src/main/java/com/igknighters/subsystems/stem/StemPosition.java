package com.igknighters.subsystems.stem;

import java.nio.ByteBuffer;
import java.util.Optional;

import com.igknighters.constants.ConstValues.kStem.kTelescope;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

@SuppressWarnings("unused")
public class StemPosition implements StructSerializable {
    public double pivotRads, wristRads, telescopeMeters;
    private boolean isStow = false;
    private Optional<String> name = Optional.empty();

    private StemPosition(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
        this.pivotRads = pivotPosRads;
        this.wristRads = wristPosRads;
        this.telescopeMeters = telescopePosMeters;
    }

    private StemPosition(double pivotPosRads, double wristPosRads, double telescopePosMeters, boolean isStow, String name) {
        this.pivotRads = pivotPosRads;
        this.wristRads = wristPosRads;
        this.telescopeMeters = telescopePosMeters;
        this.isStow = isStow;
        this.name = Optional.of(name);
    }

    public static StemPosition fromDegrees(double pivotPosDeg, double wristPosDeg, double telescopePosMeters) {
        return new StemPosition(Math.toRadians(pivotPosDeg), Math.toRadians(wristPosDeg), telescopePosMeters);
    }

    public static StemPosition fromRadians(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
        return new StemPosition(pivotPosRads, wristPosRads, telescopePosMeters);
    }

    public double getPivotRads() {
        return pivotRads;
    }

    public double getWristRads() {
        return wristRads;
    }

    public double getTelescopeMeters() {
        return telescopeMeters;
    }

    public boolean isValid() {
        return StemValidator.validatePosition(this).isValid();
    }

    public boolean isStow() {
        return false;
    }

    @Override
    public String toString() {
        if (name.isPresent()) {
            return name.get();
        } else {
            return "StemPosition(" + Units.radiansToDegrees(pivotRads) + ", " + Units.radiansToDegrees(wristRads) + ", "
                    + telescopeMeters + ")";
        }
    }

    public static class StemPositionStruct implements Struct<StemPosition> {

        @Override
        public Class<StemPosition> getTypeClass() {
            return StemPosition.class;
        }

        @Override
        public String getTypeString() {
            return "struct:StemPosition";
        }

        @Override
        public String getSchema() {
            return "double pivotRads; double wristRads; double telescopeMeters;";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 3;
        }

        @Override
        public void pack(ByteBuffer bb, StemPosition value) {
            bb.putDouble(value.pivotRads);
            bb.putDouble(value.wristRads);
            bb.putDouble(value.telescopeMeters);
        }

        @Override
        public StemPosition unpack(ByteBuffer bb) {
            return new StemPosition(bb.getDouble(), bb.getDouble(), bb.getDouble());
        }
    }

    public static final StemPositionStruct struct = new StemPositionStruct();

    public static final StemPosition STOW = new StemPosition(Units.degreesToRadians(49.0),
            Units.degreesToRadians(105.0),
            kTelescope.MIN_METERS,
            true,
            "Stow"
    );

    public static final StemPosition INTAKE = new StemPosition(Units.degreesToRadians(10.6),
            Units.degreesToRadians(73.0),
            kTelescope.MIN_METERS + Units.inchesToMeters(4.55),
            false,
            "Intake"
    );

    public static final StemPosition AMP_SAFE = new StemPosition(
            Units.degreesToRadians(90.0),
            Units.degreesToRadians(43.0),
            kTelescope.MIN_METERS + Units.inchesToMeters(2.0),
            false,
            "AmpSafe"
    );

    public static final StemPosition AMP_SCORE = new StemPosition(
            Units.degreesToRadians(88.0),
            Units.degreesToRadians(43.0),
            kTelescope.MIN_METERS + Units.inchesToMeters(7.0),
            false,
            "AmpScore"
    );

    public static final StemPosition STARTING = new StemPosition(
            1.114, // 63.8
            1.93, // 110.58
            kTelescope.MIN_METERS,
            false,
            "Starting"
    );

    public static final StemPosition DEMO_LOW = new StemPosition(
            Units.degreesToRadians(75.0),
            Units.degreesToRadians(75.0),
            kTelescope.MIN_METERS,
            false,
            "DemoLow"
    );

    public static final StemPosition PASS_STRAIGHT = new StemPosition(
            Units.degreesToRadians(75.0),
            Units.degreesToRadians(90.0),
            kTelescope.MIN_METERS,
            false,
            "DemoHigh"
    );

    public static final StemPosition SHOOTER_TEST = new StemPosition(
            Units.degreesToRadians(85.0),
            Units.degreesToRadians(95.0),
            kTelescope.MIN_METERS,
            false,
            "ShooterTest"
    );

    public static final StemPosition MANUAL_PASS = new StemPosition(
            Units.degreesToRadians(60.0),
            Units.degreesToRadians(105.0),
            kTelescope.MIN_METERS,
            false,
            "ManualPass"
    );
}