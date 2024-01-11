package com.igknighters.subsystems.stem;

public enum StemPositions {

    Stowed(50.0),
    PickupFloor(0.0),
    ScoringAmp(90.0),
    ScoringSpeakerSub(0.0);

    public final Double pivotDegrees;

    private StemPositions(Double pivotDegrees) {
        this.pivotDegrees = pivotDegrees;
    }
}
