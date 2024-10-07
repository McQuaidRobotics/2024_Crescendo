package com.igknighters.util.plumbing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

/**
 * A functional interface to aid in modifying double suppliers.
 */
@FunctionalInterface
public interface DoubleMonad extends DoubleSupplier {

    /**
     * Creates an input stream from another.
     *
     * @param base The base stream.
     * @return A new input stream.
     */
    public static DoubleMonad of(DoubleSupplier base) {
        return base::getAsDouble;
    }

    /**
     * Creates an input stream from a constant value.
     *
     * @param base The base value.
     * @return A new input stream.
     */
    public static DoubleMonad of(double base) {
        return () -> base;
    }

    public static DoubleMonad hypot(DoubleSupplier x, DoubleSupplier y) {
        return () -> Math.hypot(x.getAsDouble(), y.getAsDouble());
    }

    public static DoubleMonad atan(DoubleSupplier x, DoubleSupplier y) {
        return () -> Math.atan2(x.getAsDouble(), y.getAsDouble());
    }

    /**
     * Shorthand to return a double value.
     *
     * @return The value from {@link #getAsDouble()}.
     */
    public default double get() {
        return getAsDouble();
    }

    /**
     * Maps the stream outputs by an operator.
     *
     * @param operator A function that takes in a double input and returns a double output.
     * @return A mapped stream.
     */
    public default DoubleMonad map(DoubleUnaryOperator operator) {
        return () -> operator.applyAsDouble(getAsDouble());
    }

    /**
     * Scales the stream outputs by a factor.
     *
     * @param factor A supplier of scaling factors.
     * @return A scaled stream.
     */
    public default DoubleMonad scale(DoubleSupplier factor) {
        return map(x -> x * factor.getAsDouble());
    }

    /**
     * Scales the stream outputs by a factor.
     *
     * @param factor A scaling factor.
     * @return A scaled stream.
     */
    public default DoubleMonad scale(double factor) {
        return scale(() -> factor);
    }

    /**
     * Negates the stream outputs.
     *
     * @return A stream scaled by -1.
     */
    public default DoubleMonad negate() {
        return scale(-1);
    }

    /**
     * Offsets the stream by a factor.
     *
     * @param factor A supplier of offset values.
     * @return An offset stream.
     */
    public default DoubleMonad add(DoubleSupplier offset) {
        return map(x -> x + offset.getAsDouble());
    }

    /**
     * Offsets the stream by a factor.
     *
     * @param factor An offset.
     * @return An offset stream.
     */
    public default DoubleMonad add(double factor) {
        return add(() -> factor);
    }

    /**
     * Raises the stream outputs to an exponent.
     *
     * @param exponent The exponent to raise them to.
     * @return An exponentiated stream.
     */
    public default DoubleMonad pow(double exponent) {
        return map(x -> Math.pow(x, exponent));
    }

    /**
     * Raises the stream outputs to an exponent and keeps their original sign.
     *
     * @param exponent The exponent to raise them to.
     * @return An exponentiated stream.
     */
    public default DoubleMonad signedPow(double exponent) {
        return map(x -> Math.copySign(Math.pow(x, exponent), x));
    }

    /**
     * Filters the stream's outputs by the provided {@link LinearFilter}.
     *
     * @param filter The linear filter to use.
     * @return A filtered stream.
     */
    public default DoubleMonad filter(LinearFilter filter) {
        return map(filter::calculate);
    }

    /**
     * Deadbands the stream outputs by a minimum bound and scales them from 0 to a maximum bound.
     *
     * @param bound The lower bound to deadband with.
     * @param max The maximum value to scale with.
     * @return A deadbanded stream.
     */
    public default DoubleMonad deadband(double deadband, double max) {
        return map(x -> MathUtil.applyDeadband(x, deadband, max));
    }

    /**
     * Clamps the stream outputs by a maximum bound.
     *
     * @param magnitude The upper bound to clamp with.
     * @return A clamped stream.
     */
    public default DoubleMonad clamp(double magnitude) {
        return map(x -> MathUtil.clamp(x, -magnitude, magnitude));
    }

    /**
     * Rate limits the stream outputs by a specified rate.
     *
     * @param rate The rate in units / s.
     * @return A rate limited stream.
     */
    public default DoubleMonad rateLimit(double rate) {
        var limiter = new SlewRateLimiter(rate);
        return map(x -> limiter.calculate(x));
    }

    /**
     * Logs the output of this stream to networktables every time it is polled.
     *
     * <p>A new stream is returned that is identical to this stream, but publishes its output to
     * networktables every time it is polled.
     *
     * @param key The NetworkTables key to publish to.
     * @return A stream with the same output as this one.
     */
    public default DoubleMonad log(String key) {
        DoublePublisher pub = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
        return () -> {
            double val = this.get();
            pub.set(val);
            return val;
        };
    }

    /**
     * Gives a copy of the value to the consumer.
     * 
     * @param consumer The consumer to give the value to.
     */
    public default DoubleMonad tee(DoubleConsumer consumer) {
        return () -> {
            double val = this.get();
            consumer.accept(val);
            return val;
        };
    }
}