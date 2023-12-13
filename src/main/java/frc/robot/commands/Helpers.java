package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

public class Helpers {
    public static DoubleSupplier deadbandSupplier(DoubleSupplier input, Double db) {
        return new DoubleSupplier() {
            private final DoubleSupplier inputSupplier = input;
            private final Double deadband = db;

            @Override
            public double getAsDouble() {
                return MathUtil.applyDeadband(inputSupplier.getAsDouble(), deadband);
            }
        };
    }

    public static double clamp(Double value, Double min, Double max) {
        if (value < min) return min;
        else if (value > max) return max;
        return value;
    }

}
