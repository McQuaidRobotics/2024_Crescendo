package com.igknighters.subsystems.stem;

import com.igknighters.constants.ConstValues.kStem.kTelescope;

import edu.wpi.first.math.util.Units;

public class StemPosition {
    public double pivotRads, wristRads, telescopeMeters;

    private StemPosition(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
        this.pivotRads = pivotPosRads;
        this.wristRads = wristPosRads;
        this.telescopeMeters = telescopePosMeters;
    }

    public static StemPosition fromDegrees(double pivotPosDeg, double wristPosDeg, double telescopePosMeters) {
        return new StemPosition(Math.toRadians(pivotPosDeg), Math.toRadians(wristPosDeg), telescopePosMeters);
    }

    public static StemPosition fromRadians(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
        return new StemPosition(pivotPosRads, wristPosRads, telescopePosMeters);
    }

    public static StemPosition fromRotations(double pivotPosRot, double wristPosRot, double telescopePosMeters) {
        return new StemPosition(Math.toRadians(pivotPosRot * 360.0), Math.toRadians(wristPosRot * 360.0),
                telescopePosMeters);
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
        return StemValidator.isValidPosition(this);
    }

    public boolean isStow() {
        return false;
    }

    public static StemPosition STOW = new StemPosition(
        Units.degreesToRadians(35.0),
        kTelescope.MIN_METERS,
        Units.degreesToRadians(60.0)
    ) {
        @Override
        public boolean isValid() {
            return true;
        }

        @Override
        public String toString() {
            return "Stow";
        }

        @Override
        public boolean isStow() {
            return true;
        }
    };
}
