package org.ironmaple.configs;

import org.ironmaple.SimMechanism.Friction;
import org.ironmaple.SimMechanism.HardLimits;
import org.ironmaple.SimMechanism.MechanismDynamics;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import org.ironmaple.utils.GearRatio;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class MechanismConfig {
    public DCMotor motor;
    public MomentOfInertia rotorInertia;
    public GearRatio gearRatio;
    public Friction friction;
    public MechanismDynamics dynamics;
    public HardLimits limits;
    public double noise;

    public MechanismConfig(
            DCMotor motor,
            MomentOfInertia rotorInertia,
            GearRatio gearRatio,
            Friction friction,
            MechanismDynamics dynamics,
            HardLimits limits,
            double noise) {
        this.motor = motor;
        this.rotorInertia = rotorInertia;
        this.gearRatio = gearRatio;
        this.friction = friction;
        this.dynamics = dynamics;
        this.limits = limits;
        this.noise = noise;
    }

    public MechanismConfig(DCMotor motor) {
        this.motor = motor;
        this.rotorInertia = KilogramSquareMeters.of(0.01);
        this.gearRatio = GearRatio.reduction(1.0);
        this.friction = Friction.zero();
        this.dynamics = MechanismDynamics.zero();
        this.limits = HardLimits.unbounded();
        this.noise = 0.0;
    }

    public MechanismConfig withRotorInertia(MomentOfInertia rotorInertia) {
        this.rotorInertia = rotorInertia;
        return this;
    }

    public MechanismConfig withGearRatio(GearRatio gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public MechanismConfig withFriction(Friction friction) {
        this.friction = friction;
        return this;
    }

    public MechanismConfig withFriction(Voltage frictionVolts) {
        this.friction = Friction.of(motor, frictionVolts);
        return this;
    }

    public MechanismConfig withDynamics(MechanismDynamics dynamics) {
        this.dynamics = dynamics;
        return this;
    }

    public MechanismConfig withLimits(HardLimits limits) {
        this.limits = limits;
        return this;
    }

    public MechanismConfig withNoise(double noise) {
        this.noise = noise;
        return this;
    }
}
