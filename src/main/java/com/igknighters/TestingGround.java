package com.igknighters;

import com.igknighters.util.logging.ProceduralStructifier.ProcEnumStruct;
import com.igknighters.util.logging.ProceduralStructifier.ProcRecordStruct;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class TestingGround {
    public record MySubRecord(int int64, boolean bool, double float64) implements StructSerializable {
        public static final Struct<MySubRecord> struct = ProcRecordStruct.generate(MySubRecord.class);
    }
    public enum MySubEnum implements StructSerializable {
        A(1), B(2), C(3);

        public final int value;

        MySubEnum(int value) {
            this.value = value;
        }

        public static final Struct<MySubEnum> struct = ProcEnumStruct.generate(MySubEnum.class);
    }

    public record MyRecord(MySubRecord sub, Translation2d pose, MySubEnum eenum) {}
    private static final Struct<MyRecord> myRecordStruct = ProcRecordStruct.generate(MyRecord.class);

    private static final StructPublisher<MyRecord> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("structTestTopic", myRecordStruct)
            .publish();

    public static void periodic() {
        double rand = Math.random() * 100;
        MySubRecord subRec = new MySubRecord((int) (rand/2.0), rand > 50, rand);
        Translation2d translation = new Translation2d(-rand, rand);
        MySubEnum eenum = MySubEnum.values()[(int) (rand / 50.0)];
        MyRecord record = new MyRecord(subRec, translation, eenum);
        publisher.set(record);
    }
}