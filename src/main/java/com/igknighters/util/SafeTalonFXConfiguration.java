package com.igknighters.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SafeTalonFXConfiguration extends TalonFXConfiguration {

    public SafeTalonFXConfiguration() {
        super();
        this.CurrentLimits.StatorCurrentLimit = 60;
        this.CurrentLimits.StatorCurrentLimitEnable = true;
        this.CurrentLimits.SupplyCurrentLimit = 25;
        this.CurrentLimits.SupplyCurrentThreshold = 40;
        this.CurrentLimits.SupplyTimeThreshold = 0.15;
    }
}
