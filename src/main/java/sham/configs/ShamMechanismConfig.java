package sham.configs;

import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import sham.utils.GearRatio;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public class ShamMechanismConfig {
    public DCMotor motor;
    public MomentOfInertia rotorInertia;
    public GearRatio gearRatio;
    public Friction friction;
    public MechanismDynamics dynamics;
    public HardLimits limits;
    public double noise;

    public ShamMechanismConfig(
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

    public ShamMechanismConfig(DCMotor motor) {
        this.motor = motor;
        this.rotorInertia = KilogramSquareMeters.of(0.01);
        this.gearRatio = GearRatio.reduction(1.0);
        this.friction = Friction.zero();
        this.dynamics = MechanismDynamics.zero();
        this.limits = HardLimits.unbounded();
        this.noise = 0.0;
    }

    public ShamMechanismConfig withRotorInertia(MomentOfInertia rotorInertia) {
        this.rotorInertia = rotorInertia;
        return this;
    }

    public ShamMechanismConfig withGearRatio(GearRatio gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public ShamMechanismConfig withFriction(Friction friction) {
        this.friction = friction;
        return this;
    }

    public ShamMechanismConfig withFriction(Voltage frictionVolts) {
        this.friction = Friction.of(motor, frictionVolts);
        return this;
    }

    public ShamMechanismConfig withDynamics(MechanismDynamics dynamics) {
        this.dynamics = dynamics;
        return this;
    }

    public ShamMechanismConfig withLimits(HardLimits limits) {
        this.limits = limits;
        return this;
    }

    public ShamMechanismConfig withNoise(double noise) {
        this.noise = noise;
        return this;
    }
}
