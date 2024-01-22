package com.igknighters.subsystems.stem;

public class StemPositions {
    public static class StemPosition {
        public double pivotPosRads, wristPosRads, telescopePosMeters;

        private StemPosition(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
            this.pivotPosRads = pivotPosRads;
            this.wristPosRads = wristPosRads;
            this.telescopePosMeters = telescopePosMeters;
        }

        public static StemPosition fromDegrees(double pivotPosDeg, double wristPosDeg, double telescopePosMeters) {
            return new StemPosition(Math.toRadians(pivotPosDeg), Math.toRadians(wristPosDeg), telescopePosMeters);
        }

        public static StemPosition fromRadians(double pivotPosRads, double wristPosRads, double telescopePosMeters) {
            return new StemPosition(pivotPosRads, wristPosRads, telescopePosMeters);
        }

        public static StemPosition fromRotations(double pivotPosRot, double wristPosRot, double telescopePosMeters) {
            return new StemPosition(Math.toRadians(pivotPosRot * 360.0), Math.toRadians(wristPosRot * 360.0), telescopePosMeters);
        }
    }

}
