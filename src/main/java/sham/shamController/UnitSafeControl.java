package sham.shamController;



import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import sham.utils.mathutils.MeasureMath;

public class UnitSafeControl {
    /**
     * A PD controller that uses units to ensure that the controller is used correctly.
     */
    public static class PDFeedback<O extends Unit, Q extends Unit> {
        private final edu.wpi.first.math.controller.PIDController internalController;
        private final O outputUnit;

        public PDFeedback(
                Per<O, Q> kP,
                Per<O, VelocityUnit<Q>> kD) {
            outputUnit = kP.unit().numerator();
            internalController = new edu.wpi.first.math.controller.PIDController(kP.baseUnitMagnitude(), 0.0, kD.baseUnitMagnitude());
        }

        public PDFeedback(
                Per<O, Q> kP) {
            outputUnit = kP.unit().numerator();
            internalController = new edu.wpi.first.math.controller.PIDController(kP.baseUnitMagnitude(), 0.0, 0.0);
        }

        @SuppressWarnings("unchecked")
        public Measure<O> calculate(Measure<Q> measurement, Measure<Q> setpoint) {
            return (Measure<O>) outputUnit.of(
                    internalController.calculate(measurement.baseUnitMagnitude(), setpoint.baseUnitMagnitude()));
        }

        public PDFeedback<O, Q> withTolerance(Measure<Q> tolerance) {
            internalController.setTolerance(tolerance.baseUnitMagnitude());
            return this;
        }

        public PDFeedback<O, Q> withTolerance(Measure<Q> positionTolerance, Measure<Q> velocityTolerance) {
            internalController.setTolerance(positionTolerance.baseUnitMagnitude(), velocityTolerance.baseUnitMagnitude());
            return this;
        }

        public PDFeedback<O, Q> withContinuousInput(Measure<Q> minimumInput, Measure<Q> maximumInput) {
            internalController.enableContinuousInput(minimumInput.baseUnitMagnitude(), maximumInput.baseUnitMagnitude());
            return this;
        }
    }

    public static class LinearPDFeedback<O extends Unit> extends PDFeedback<O, DistanceUnit> {
        public LinearPDFeedback(
                Per<O, DistanceUnit> kP,
                Per<O, PerUnit<DistanceUnit, TimeUnit>> kD) {
            super(
                kP,
                PerUnit.combine(
                    kD.unit().numerator(),
                    VelocityUnit.combine(Meters, Second)
                ).ofNative(kD.baseUnitMagnitude())
            );
        }

        public LinearPDFeedback(
                Per<O, DistanceUnit> kP) {
            super(kP);
        }

        public Measure<O> calculate(Distance measurement, Distance setpoint) {
            return super.calculate(measurement, setpoint);
        }
    }

    public static class AngularPDFeedback<O extends Unit> extends PDFeedback<O, AngleUnit> {
        public AngularPDFeedback(
                Per<O, AngleUnit> kP,
                Per<O, AngularVelocityUnit> kD) {
            super(
                kP,
                PerUnit.combine(
                    kD.unit().numerator(),
                    VelocityUnit.combine(Radian, Second)
                ).ofNative(kD.baseUnitMagnitude())
            );
        }

        public AngularPDFeedback(
                Per<O, AngleUnit> kP) {
            super(kP);
        }

        public Measure<O> calculate(Angle measurement, Angle setpoint) {
            return super.calculate(measurement, setpoint);
        }

        public AngularPDFeedback<O> withContinuousAngularInput() {
            super.withContinuousInput(Radian.of(-Math.PI), Radian.of(Math.PI));
            return this;
        }
    }

    public static class AngularVelocityPDFeedback<O extends Unit> extends PDFeedback<O, AngularVelocityUnit> {
        public AngularVelocityPDFeedback(
                Per<O, AngularVelocityUnit> kP,
                Per<O, AngularAccelerationUnit> kD) {
            super(
                kP,
                PerUnit.combine(
                    kD.unit().numerator(),
                    VelocityUnit.combine(RadiansPerSecond, Second)
                ).ofNative(kD.baseUnitMagnitude())
            );
        }

        public AngularVelocityPDFeedback(
                Per<O, AngularVelocityUnit> kP) {
            super(kP);
        }

        public Measure<O> calculate(AngularVelocity measurement, AngularVelocity setpoint) {
            return super.calculate(measurement, setpoint);
        }
    }

    public interface Feedforward<O extends Unit, Q extends Unit> {
        Measure<O> universalCalculate(
                Measure<Q> goal,
                Velocity<Q> goalRate,
                Acceleration<Q> goalRateRate,
                Measure<Q> state,
                Velocity<Q> stateRate,
                Acceleration<Q> stateRateRate);
    }

    public static class FlywheelFeedforward<O extends Unit> implements Feedforward<O, AngleUnit> {
        private final edu.wpi.first.math.controller.SimpleMotorFeedforward internalFeedforward;
        private final O outputUnit;

        public FlywheelFeedforward(
                Measure<O> kS,
                Per<O, AngularVelocityUnit> kV,
                Per<O, AngularAccelerationUnit> kA) {
            outputUnit = kS.unit();
            internalFeedforward = new edu.wpi.first.math.controller.SimpleMotorFeedforward(
                    kS.baseUnitMagnitude(),
                    kV.baseUnitMagnitude(),
                    kA.baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(AngularVelocity goalRate, AngularAcceleration goalRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
        }

        @SuppressWarnings("unchecked")
        public Measure<O> calculate(AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate, nextGoalRate).baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(AngularVelocity goalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
        }

        @Override
        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> universalCalculate(
                Measure<AngleUnit> goal,
                Velocity<AngleUnit> goalRate,
                Acceleration<AngleUnit> goalRateRate,
                Measure<AngleUnit> state,
                Velocity<AngleUnit> stateRate,
                Acceleration<AngleUnit> stateRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
        }
    }

    public static class ElevatorFeedforward<O extends Unit> implements Feedforward<O, DistanceUnit> {
        private final edu.wpi.first.math.controller.ElevatorFeedforward internalFeedforward;
        private final O outputUnit;

        public ElevatorFeedforward(
                Measure<O> kS,
                Per<O, VelocityUnit<DistanceUnit>> kV,
                Per<O, AccelerationUnit<DistanceUnit>> kA) {
            outputUnit = kS.unit();
            internalFeedforward = new edu.wpi.first.math.controller.ElevatorFeedforward(
                    kS.baseUnitMagnitude(),
                    kV.baseUnitMagnitude(),
                    kA.baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(LinearVelocity goalRate, LinearAcceleration goalRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
        }

        @SuppressWarnings("unchecked")
        public Measure<O> calculate(LinearVelocity goalRate, LinearVelocity nextGoalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate, nextGoalRate).baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(LinearVelocity goalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude()));
        }

        @Override
        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> universalCalculate(
                Measure<DistanceUnit> goal,
                Velocity<DistanceUnit> goalRate,
                Acceleration<DistanceUnit> goalRateRate,
                Measure<DistanceUnit> state,
                Velocity<DistanceUnit> stateRate,
                Acceleration<DistanceUnit> stateRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(goalRate.baseUnitMagnitude(), goalRateRate.baseUnitMagnitude()));
        }
    }

    public static class ArmFeedforward<O extends Unit> implements Feedforward<O, AngleUnit> {
        private final edu.wpi.first.math.controller.ArmFeedforward internalFeedforward;
        private final O outputUnit;

        public ArmFeedforward(
                Measure<O> kS,
                Per<O, VelocityUnit<AngleUnit>> kV,
                Per<O, AccelerationUnit<AngleUnit>> kA) {
            outputUnit = kS.unit();
            internalFeedforward = new edu.wpi.first.math.controller.ArmFeedforward(
                    kS.baseUnitMagnitude(),
                    kV.baseUnitMagnitude(),
                    kA.baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate,
                Acceleration<AngleUnit> goalRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(currentAngle.baseUnitMagnitude(), goalRate.baseUnitMagnitude(),
                            goalRateRate.baseUnitMagnitude()));
        }

        @SuppressWarnings("unchecked")
        public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate, AngularVelocity nextGoalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(currentAngle, goalRate, nextGoalRate).baseUnitMagnitude());
        }

        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> calculate(Angle currentAngle, AngularVelocity goalRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(currentAngle.baseUnitMagnitude(), goalRate.baseUnitMagnitude()));
        }

        @Override
        @SuppressWarnings({ "unchecked", "removal" })
        public Measure<O> universalCalculate(
                Measure<AngleUnit> goal,
                Velocity<AngleUnit> goalRate,
                Acceleration<AngleUnit> goalRateRate,
                Measure<AngleUnit> state,
                Velocity<AngleUnit> stateRate,
                Acceleration<AngleUnit> stateRateRate) {
            return (Measure<O>) outputUnit.of(
                    internalFeedforward.calculate(
                            state.baseUnitMagnitude(),
                            goalRate.baseUnitMagnitude(),
                            goalRateRate.baseUnitMagnitude()));
        }
    }

    /** {@link TrapezoidProfile} */
    public static class TrapezoidProfile<DIM extends Unit> {
        private final edu.wpi.first.math.trajectory.TrapezoidProfile internalProfile;
        private final Measure<DIM> maxValue;

        public record State<DIM extends Unit> (
            Measure<DIM> value,
            Velocity<DIM> slew
        ) {
            public static State<AngleUnit> of(Angle position, AngularVelocity velocity) {
                return new State<>(
                    position,
                    VelocityUnit.combine(Radian, Second).of(velocity.in(RadiansPerSecond))
                );
            }

            public static State<DistanceUnit> of(Distance position, LinearVelocity velocity) {
                return new State<>(
                    position,
                    VelocityUnit.combine(Meters, Second).of(velocity.in(MetersPerSecond))
                );
            }

            public static State<AngularVelocityUnit> of(AngularVelocity velocity, AngularAcceleration acceleration) {
                return new State<>(
                    velocity,
                    VelocityUnit.combine(RadiansPerSecond, Second).of(acceleration.in(RadiansPerSecondPerSecond))
                );
            }

            public static State<LinearVelocityUnit> of(LinearVelocity velocity, LinearAcceleration acceleration) {
                return new State<>(
                    velocity,
                    VelocityUnit.combine(MetersPerSecond, Second).of(acceleration.in(MetersPerSecondPerSecond))
                );
            }
        }

        private TrapezoidProfile(
            Measure<DIM> maxValue,
            Velocity<DIM> maxSlew,
            Acceleration<DIM> maxSlewSlew
        ) {
            this.maxValue = maxValue;
            internalProfile = new edu.wpi.first.math.trajectory.TrapezoidProfile(
                new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
                    maxSlew.baseUnitMagnitude(),
                    maxSlewSlew.baseUnitMagnitude()
                )
            );
        }

        public static TrapezoidProfile<AngleUnit> forAngle(
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration
        ) {
            final VelocityUnit<AngleUnit> vu = VelocityUnit.combine(Radian, Second);
            return new TrapezoidProfile<>(
                Radian.of(10000000000.0),
                vu.of(maxVelocity.in(RadiansPerSecond)),
                AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(RadiansPerSecondPerSecond))
            );
        }

        public static TrapezoidProfile<DistanceUnit> forDistance(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration
        ) {
            final VelocityUnit<DistanceUnit> vu = VelocityUnit.combine(Meters, Second);
            return new TrapezoidProfile<>(
                Meters.of(10000000000.0),
                vu.of(maxVelocity.in(MetersPerSecond)),
                AccelerationUnit.combine(vu, Second).of(maxAcceleration.in(MetersPerSecondPerSecond))
            );
        }

        public static TrapezoidProfile<AngularVelocityUnit> forAngularVelocity(
            AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration
        ) {
            final VelocityUnit<AngularVelocityUnit> vu = VelocityUnit.combine(RadiansPerSecond, Second);
            return new TrapezoidProfile<>(
                maxVelocity,
                vu.of(maxAcceleration.in(RadiansPerSecondPerSecond)),
                AccelerationUnit.combine(vu, Second).of(10000000.0)
            );
        }

        public static TrapezoidProfile<LinearVelocityUnit> forLinearVelocity(
            LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration
        ) {
            final VelocityUnit<LinearVelocityUnit> vu = VelocityUnit.combine(MetersPerSecond, Second);
            return new TrapezoidProfile<>(
                maxVelocity,
                vu.of(maxAcceleration.in(MetersPerSecondPerSecond)),
                AccelerationUnit.combine(vu, Second).of(10000000.0)
            );
        }

        @SuppressWarnings("unchecked")
        public State<DIM> calculate(
            State<DIM> current,
            State<DIM> goal,
            Time deltaTime
        ) {
            var internalState = internalProfile.calculate(
                deltaTime.baseUnitMagnitude(),
                new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                    current.value.baseUnitMagnitude(),
                    current.slew.baseUnitMagnitude()
                ),
                new edu.wpi.first.math.trajectory.TrapezoidProfile.State(
                    goal.value.baseUnitMagnitude(),
                    goal.slew.baseUnitMagnitude()
                )
            );

            Measure<DIM> value = (Measure<DIM>) current.value.unit().of(internalState.position);
            Velocity<DIM> slew = VelocityUnit.combine(value.unit(), Second).of(internalState.velocity);
            if (MeasureMath.abs(value).gt(maxValue)) {
                value = MeasureMath.clamp(value, maxValue);
                return new State<>(value, (Velocity<DIM>) slew.unit().zero());
            } else {
                return new State<>(
                    value,
                    slew
                );
            }
        }
    }
}
