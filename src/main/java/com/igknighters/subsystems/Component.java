package com.igknighters.subsystems;

import monologue.LogLocal;

public abstract class Component implements LogLocal {

    /**
     * Should be called every cycle in the parent subsystems periodic method
     */
    public void periodic() {};
}
