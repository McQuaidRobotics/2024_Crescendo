package com.igknighters.subsystems.stem;

public class StemPositions {
    public static final class StemPosition {
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
    }

}
