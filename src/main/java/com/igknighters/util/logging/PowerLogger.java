package com.igknighters.util.logging;

import java.nio.ByteBuffer;
import java.util.function.Consumer;

import com.igknighters.Robot;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionJNI;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * A utility to log data from the PDP/PDH with as little overhead as possible.
 */
public class PowerLogger {
    private final PD pd;
    private final PDData data;
    private final Consumer<PDData> output;
    private boolean initialized = false;

    /**
     * Creates a new PowerLogger.
     * 
     * @param canId The CAN ID of the PDP/PDH
     * @param moduleType The module type of the PDP/PDH
     * @param path The path to log to
     * @param datalogOnly Whether to only log to the datalog
     */
    public PowerLogger(
        int canId,
        ModuleType moduleType,
        String path,
        boolean datalogOnly
    ) {
        int handle;
        int module;
        try {
            handle = Robot.isReal()
                ? PowerDistributionJNI.initialize(canId, moduleType.value)
                : PowerDistributionJNI.initialize(-1, 0);
            module = PowerDistributionJNI.getModuleNumber(handle);

            initialized = true;
        } catch (Exception e) {
            DriverStation.reportError("Error initializing power logger: " + e.getMessage(), e.getStackTrace());
            pd = null;
            data = null;
            output = null;
            return;
        }
        pd = new PD(handle, module);
        data = new PDData(pd);

        if (!datalogOnly) {
            StructEntry<PDData> entry = NetworkTableInstance.getDefault()
                    .getStructTopic("PowerDistribution/", new PDDataStruct())
                    .getEntry(data);
            output = entry::set;
        } else {
            StructLogEntry<PDData> entry = StructLogEntry.create(
                DataLogManager.getLog(),
                path,
                new PDDataStruct()
            );
            output = entry::append;
        }
    }

    /**
     * Logs the the most recent data, should be called periodically.
     */
    public void log() {
        if (!initialized) {
            return;
        }
        try {
            data.update(pd);
            output.accept(data);
        } catch (Exception e) {
            DriverStation.reportError("Error logging power data: " + e.getMessage(), e.getStackTrace());
        }
    }

    protected static final record PD(int handle, int module) {
        public int getStickyFaults() {
            return PowerDistributionJNI.getStickyFaultsNative(handle);
        }

        public int getFaults() {
            return PowerDistributionJNI.getFaultsNative(handle);
        }

        public boolean getSwitchableChannel() {
            return PowerDistributionJNI.getSwitchableChannel(handle);
        }

        public double getVoltage() {
            return PowerDistributionJNI.getVoltage(handle);
        }

        public double getTotalCurrent() {
            return PowerDistributionJNI.getTotalCurrent(handle);
        }

        public double getTemperature() {
            return PowerDistributionJNI.getTemperature(handle);
        }

        public void getAllCurrents(double[] outCurrents) {
            PowerDistributionJNI.getAllCurrents(handle, outCurrents);
        }
    }


    protected static final class PowerDistributionFaultsStruct implements Struct<PowerDistributionFaults> {
        @Override
        public Class<PowerDistributionFaults> getTypeClass() {
            return PowerDistributionFaults.class;
        }

        @Override
        public int getSize() {
            return 4; //doing bitfields on a u32
        }

        @Override
        public String getSchema() {
            return "uint32 Channel0BreakerFault:1; "
                + "uint32 Channel1BreakerFault:1; "
                + "uint32 Channel2BreakerFault:1; "
                + "uint32 Channel3BreakerFault:1; "
                + "uint32 Channel4BreakerFault:1; "
                + "uint32 Channel5BreakerFault:1; "
                + "uint32 Channel6BreakerFault:1; "
                + "uint32 Channel7BreakerFault:1; "
                + "uint32 Channel8BreakerFault:1; "
                + "uint32 Channel9BreakerFault:1; "
                + "uint32 Channel10BreakerFault:1; "
                + "uint32 Channel11BreakerFault:1; "
                + "uint32 Channel12BreakerFault:1; "
                + "uint32 Channel13BreakerFault:1; "
                + "uint32 Channel14BreakerFault:1; "
                + "uint32 Channel15BreakerFault:1; "
                + "uint32 Channel16BreakerFault:1; "
                + "uint32 Channel17BreakerFault:1; "
                + "uint32 Channel18BreakerFault:1; "
                + "uint32 Channel19BreakerFault:1; "
                + "uint32 Channel20BreakerFault:1; "
                + "uint32 Channel21BreakerFault:1; "
                + "uint32 Channel22BreakerFault:1; "
                + "uint32 Channel23BreakerFault:1; "
                + "uint32 Brownout:1; "
                + "uint32 CanWarning:1; "
                + "uint32 HardwareFault:1; ";
        }

        @Override
        public String getTypeString() {
            return "struct:PowerDistributionFaults";
        }

        @Override
        public void pack(ByteBuffer bb, PowerDistributionFaults value) {
            int packed = 0;
            packed |= value.Channel0BreakerFault ? 1 : 0;
            packed |= value.Channel1BreakerFault ? 1 << 1 : 0;
            packed |= value.Channel2BreakerFault ? 1 << 2 : 0;
            packed |= value.Channel3BreakerFault ? 1 << 3 : 0;
            packed |= value.Channel4BreakerFault ? 1 << 4 : 0;
            packed |= value.Channel5BreakerFault ? 1 << 5 : 0;
            packed |= value.Channel6BreakerFault ? 1 << 6 : 0;
            packed |= value.Channel7BreakerFault ? 1 << 7 : 0;
            packed |= value.Channel8BreakerFault ? 1 << 8 : 0;
            packed |= value.Channel9BreakerFault ? 1 << 9 : 0;
            packed |= value.Channel10BreakerFault ? 1 << 10 : 0;
            packed |= value.Channel11BreakerFault ? 1 << 11 : 0;
            packed |= value.Channel12BreakerFault ? 1 << 12 : 0;
            packed |= value.Channel13BreakerFault ? 1 << 13 : 0;
            packed |= value.Channel14BreakerFault ? 1 << 14 : 0;
            packed |= value.Channel15BreakerFault ? 1 << 15 : 0;
            packed |= value.Channel16BreakerFault ? 1 << 16 : 0;
            packed |= value.Channel17BreakerFault ? 1 << 17 : 0;
            packed |= value.Channel18BreakerFault ? 1 << 18 : 0;
            packed |= value.Channel19BreakerFault ? 1 << 19 : 0;
            packed |= value.Channel20BreakerFault ? 1 << 20 : 0;
            packed |= value.Channel21BreakerFault ? 1 << 21 : 0;
            packed |= value.Channel22BreakerFault ? 1 << 22 : 0;
            packed |= value.Channel23BreakerFault ? 1 << 23 : 0;
            packed |= value.Brownout ? 1 << 24 : 0;
            packed |= value.CanWarning ? 1 << 25 : 0;
            packed |= value.HardwareFault ? 1 << 26 : 0;

            bb.putInt(packed);
        }

        public void pack(ByteBuffer bb, int value) {
            bb.putInt(value);
        }

        @Override
        public PowerDistributionFaults unpack(ByteBuffer bb) {
            int packed = bb.getInt();
            return new PowerDistributionFaults(packed);
        }
    }

    protected static final class PowerDistributionStickyFaultsStruct implements Struct<PowerDistributionStickyFaults> {
        @Override
        public Class<PowerDistributionStickyFaults> getTypeClass() {
            return PowerDistributionStickyFaults.class;
        }

        @Override
        public int getSize() {
            return 4; //doing bitfields on a u32
        }

        @Override
        public String getSchema() {
            return "uint32 Channel0BreakerFault:1; "
                + "uint32 Channel1BreakerFault:1; "
                + "uint32 Channel2BreakerFault:1; "
                + "uint32 Channel3BreakerFault:1; "
                + "uint32 Channel4BreakerFault:1; "
                + "uint32 Channel5BreakerFault:1; "
                + "uint32 Channel6BreakerFault:1; "
                + "uint32 Channel7BreakerFault:1; "
                + "uint32 Channel8BreakerFault:1; "
                + "uint32 Channel9BreakerFault:1; "
                + "uint32 Channel10BreakerFault:1; "
                + "uint32 Channel11BreakerFault:1; "
                + "uint32 Channel12BreakerFault:1; "
                + "uint32 Channel13BreakerFault:1; "
                + "uint32 Channel14BreakerFault:1; "
                + "uint32 Channel15BreakerFault:1; "
                + "uint32 Channel16BreakerFault:1; "
                + "uint32 Channel17BreakerFault:1; "
                + "uint32 Channel18BreakerFault:1; "
                + "uint32 Channel19BreakerFault:1; "
                + "uint32 Channel20BreakerFault:1; "
                + "uint32 Channel21BreakerFault:1; "
                + "uint32 Channel22BreakerFault:1; "
                + "uint32 Channel23BreakerFault:1; "
                + "uint32 Brownout:1; "
                + "uint32 CanWarning:1; "
                + "uint32 CanBusOff:1; "
                + "uint32 HasReset:1; ";
        }

        @Override
        public String getTypeString() {
            return "struct:PowerDistributionStickyFaults";
        }

        @Override
        public void pack(ByteBuffer bb, PowerDistributionStickyFaults value) {
            int packed = 0;
            packed |= value.Channel0BreakerFault ? 1 : 0;
            packed |= value.Channel1BreakerFault ? 1 << 1 : 0;
            packed |= value.Channel2BreakerFault ? 1 << 2 : 0;
            packed |= value.Channel3BreakerFault ? 1 << 3 : 0;
            packed |= value.Channel4BreakerFault ? 1 << 4 : 0;
            packed |= value.Channel5BreakerFault ? 1 << 5 : 0;
            packed |= value.Channel6BreakerFault ? 1 << 6 : 0;
            packed |= value.Channel7BreakerFault ? 1 << 7 : 0;
            packed |= value.Channel8BreakerFault ? 1 << 8 : 0;
            packed |= value.Channel9BreakerFault ? 1 << 9 : 0;
            packed |= value.Channel10BreakerFault ? 1 << 10 : 0;
            packed |= value.Channel11BreakerFault ? 1 << 11 : 0;
            packed |= value.Channel12BreakerFault ? 1 << 12 : 0;
            packed |= value.Channel13BreakerFault ? 1 << 13 : 0;
            packed |= value.Channel14BreakerFault ? 1 << 14 : 0;
            packed |= value.Channel15BreakerFault ? 1 << 15 : 0;
            packed |= value.Channel16BreakerFault ? 1 << 16 : 0;
            packed |= value.Channel17BreakerFault ? 1 << 17 : 0;
            packed |= value.Channel18BreakerFault ? 1 << 18 : 0;
            packed |= value.Channel19BreakerFault ? 1 << 19 : 0;
            packed |= value.Channel20BreakerFault ? 1 << 20 : 0;
            packed |= value.Channel21BreakerFault ? 1 << 21 : 0;
            packed |= value.Channel22BreakerFault ? 1 << 22 : 0;
            packed |= value.Channel23BreakerFault ? 1 << 23 : 0;
            packed |= value.Brownout ? 1 << 24 : 0;
            packed |= value.CanWarning ? 1 << 25 : 0;
            packed |= value.CanBusOff ? 1 << 26 : 0;
            packed |= value.HasReset ? 1 << 27 : 0;

            bb.putInt(packed);
        }

        public void pack(ByteBuffer bb, int value) {
            bb.putInt(value);
        }

        @Override
        public PowerDistributionStickyFaults unpack(ByteBuffer bb) {
            int packed = bb.getInt();
            return new PowerDistributionStickyFaults(packed);
        }
    }

    protected static final class PDData {
        public int faults;
        public int stickyFaults;
        public double voltage;
        public double totalCurrent;
        public boolean switchableChannel;
        public double temperature;
        public double[] currents;

        public PDData(final PD pd) {
            currents = new double[24];
            update(pd);
        }

        public void update(final PD pd) {
            faults = pd.getFaults();
            stickyFaults = pd.getStickyFaults();
            voltage = pd.getVoltage();
            totalCurrent = pd.getTotalCurrent();
            switchableChannel = pd.getSwitchableChannel();
            temperature = pd.getTemperature();
            pd.getAllCurrents(currents);
        }
    }

    protected static final class PDDataStruct implements Struct<PDData> {
        @Override
        public Class<PDData> getTypeClass() {
            return PDData.class;
        }

        @Override
        public int getSize() {
            return 4 + 4 + 8 + 8 + 1 + 8 + 8 * 24;
        }

        @Override
        public String getSchema() {
            return "PowerDistributionFaults faults; "
                + "PowerDistributionStickyFaults stickyFaults; "
                + "double voltage; "
                + "double totalCurrent; "
                + "bool switchableChannel; "
                + "double temperature; "
                + "double currents[24]; ";

        }

        @Override
        public String getTypeString() {
            return "struct:PDData";
        }

        @Override
        public void pack(ByteBuffer bb, PDData value) {
            bb.putInt(value.faults);
            bb.putInt(value.stickyFaults);
            bb.putDouble(value.voltage);
            bb.putDouble(value.totalCurrent);
            bb.put((byte) (value.switchableChannel ? 1 : 0));
            bb.putDouble(value.temperature);
            for (int i = 0; i < 24; i++) {
                bb.putDouble(value.currents[i]);
            }
        }

        @Override
        public PDData unpack(ByteBuffer bb) {
            PDData data = new PDData(null);
            data.faults = bb.getInt();
            data.stickyFaults = bb.getInt();
            data.voltage = bb.getDouble();
            data.totalCurrent = bb.getDouble();
            data.switchableChannel = bb.get() == 1;
            data.temperature = bb.getDouble();
            for (int i = 0; i < 24; i++) {
                data.currents[i] = bb.getDouble();
            }
            return data;
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {
                new PowerDistributionFaultsStruct(),
                new PowerDistributionStickyFaultsStruct()
            };
        }
    }
}
