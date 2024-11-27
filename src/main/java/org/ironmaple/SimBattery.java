package org.ironmaple;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

public class SimBattery {
    private final ConcurrentLinkedQueue<Supplier<Current>> electricalAppliances = new ConcurrentLinkedQueue<>();

    public void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        this.electricalAppliances.add(customElectricalAppliances);
    }

    public void addMechanism(SimMechanism simMechanism) {
        this.electricalAppliances.add(() -> simMechanism.inputs().supplyCurrent());
    }

    public Voltage getBatteryVoltage() {
        final double[] totalCurrentAmps = electricalAppliances.stream()
                .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
                .toArray();

        double batteryVoltageVolts = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);

        return Volts.of(MathUtil.clamp(batteryVoltageVolts, 0, 12));
    }
}
