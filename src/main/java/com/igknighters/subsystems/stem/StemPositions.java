package com.igknighters.subsystems.stem;

public class StemPositions {

    public static class StemPosition {
        public double pivotPos, wristPos, telescopePos;

        public StemPosition(double pivotPos, double wristPos, double telescopePos){
            this.pivotPos = pivotPos;
            this.wristPos = wristPos;
            this.telescopePos = telescopePos;
        }
    }

}
