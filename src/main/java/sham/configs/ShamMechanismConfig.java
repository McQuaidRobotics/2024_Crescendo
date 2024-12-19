package sham.configs;

import sham.ShamMechanism.Friction;
import sham.ShamMechanism.HardLimits;
import sham.ShamMechanism.MechanismDynamics;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import sham.utils.GearRatio;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.procstruct.ProceduralStructGenerator;
import monologue.procstruct.ProceduralStructGenerator.IgnoreStructField;

public class ShamMechanismConfig implements StructSerializable {
    public DCMotor motor;
    public MomentOfInertia rotorInertia;
    public GearRatio gearRatio;
    public Friction friction;
    @IgnoreStructField
    public MechanismDynamics dynamics;
    public HardLimits limits;
    public Velocity<VoltageUnit> voltageRamp;
    public double noise;

    public ShamMechanismConfig(
            DCMotor motor,
            MomentOfInertia rotorInertia,
            GearRatio gearRatio,
            Friction friction,
            MechanismDynamics dynamics,
            HardLimits limits,
            Velocity<VoltageUnit> voltageRamp,
            double noise) {
        this.motor = motor;
        this.rotorInertia = rotorInertia;
        this.gearRatio = gearRatio;
        this.friction = friction;
        this.dynamics = dynamics;
        this.limits = limits;
        this.voltageRamp = voltageRamp;
        this.noise = noise;
    }

    public ShamMechanismConfig(DCMotor motor) {
        this.motor = motor;
        this.rotorInertia = KilogramSquareMeters.of(0.01);
        this.gearRatio = GearRatio.reduction(1.0);
        this.friction = Friction.zero();
        this.dynamics = MechanismDynamics.zero();
        this.limits = HardLimits.unbounded();
        this.voltageRamp = Volts.of(600.0).per(Second);
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

    public ShamMechanismConfig withFriction(Voltage staticFrictionVolts, Voltage kineticFrictionVoltage) {
        this.friction = Friction.of(motor, staticFrictionVolts, kineticFrictionVoltage);
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

    public ShamMechanismConfig withVoltageRamp(Velocity<VoltageUnit> voltageRamp) {
        this.voltageRamp = voltageRamp;
        return this;
    }

    public ShamMechanismConfig withNoise(double noise) {
        this.noise = noise;
        return this;
    }

    public static final Struct<ShamMechanismConfig> struct = ProceduralStructGenerator.genObjectNoUnpack(ShamMechanismConfig.class);
}
