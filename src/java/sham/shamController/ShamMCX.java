package sham.shamController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.util.Optional;
import monologue.ProceduralStructGenerator;
import sham.ShamMechanism.MechanismState;
import sham.ShamMotorController;
import sham.ShamMotorController.ControllerOutput.CurrentOutput;
import sham.ShamMotorController.ControllerOutput.VoltageOutput;
import sham.shamController.UnitSafeControl.TrapezoidProfile.State;
import sham.utils.RuntimeLog;
import wpilibExt.DCMotorExt;
import wpilibExt.MeasureMath;

public class ShamMCX implements ShamMotorController {
  private static final VelocityUnit<AngleUnit> VU = VelocityUnit.combine(Radians, Seconds);
  private static final VelocityUnit<AngularVelocityUnit> AU =
      VelocityUnit.combine(RadiansPerSecond, Seconds);

  private sealed interface Output {
    public ControllerOutput run(
        Time dt, Voltage supply, MechanismState state, EpilogueBackend logger);

    public record ClosedLoopOutput<U extends Unit>(
        ClosedLoop<?, U, AngleUnit> controller, Measure<U> value, Velocity<U> secondOrderValue)
        implements Output {
      @Override
      @SuppressWarnings("unchecked")
      public ControllerOutput run(
          Time dt, Voltage supply, MechanismState state, EpilogueBackend logger) {
        Measure<?> output;
        if (controller().isVelocity()) {
          output =
              controller()
                  .runVelocity(
                      state.position(),
                      State.of(state.velocity(), state.acceleration()),
                      new State<>(
                          (Measure<AngularVelocityUnit>) value,
                          (Velocity<AngularVelocityUnit>) secondOrderValue),
                      dt,
                      logger);
        } else {
          output =
              controller()
                  .runPosition(
                      state.position(),
                      State.of(state.position(), state.velocity()),
                      new State<>(
                          (Measure<AngleUnit>) value, (Velocity<AngleUnit>) secondOrderValue),
                      dt,
                      logger);
        }
        if (output.unit() instanceof VoltageUnit) {
          return ControllerOutput.of((Voltage) output);
        } else {
          return ControllerOutput.of((Current) output);
        }
      }
    }

    public record OpenLoopVoltageOutput(Voltage volts) implements Output {
      @Override
      public ControllerOutput run(
          Time dt, Voltage supply, MechanismState state, EpilogueBackend logger) {
        return ControllerOutput.of(MeasureMath.clamp(volts, supply));
      }
    }

    public record OpenLoopCurrentOutput(Current amps) implements Output {
      @Override
      public ControllerOutput run(
          Time dt, Voltage supply, MechanismState state, EpilogueBackend logger) {
        return ControllerOutput.of(amps);
      }
    }

    public static OpenLoopVoltageOutput of(Voltage volts) {
      return new OpenLoopVoltageOutput(volts);
    }

    public static OpenLoopCurrentOutput of(Current amps) {
      return new OpenLoopCurrentOutput(amps);
    }
  }

  public record CurrentLimits(
      Current statorCurrentLimit,
      Current supplyCurrentLimit,
      Current supplyCurrentLowerLimit,
      Time lowerLimitTriggerTime)
      implements StructSerializable {
    public static CurrentLimits base() {
      return new CurrentLimits(Amps.of(120.0), Amps.of(70.0), Amps.of(40.0), Seconds.of(1.0));
    }

    public CurrentLimits times(double factor) {
      return new CurrentLimits(
          statorCurrentLimit.times(factor),
          supplyCurrentLimit.times(factor),
          supplyCurrentLowerLimit.times(factor),
          lowerLimitTriggerTime);
    }

    public static final Struct<CurrentLimits> struct =
        ProceduralStructGenerator.genRecord(CurrentLimits.class);
  }

  public enum SoftLimitTrigger {
    NONE,
    REVERSE,
    FORWARD;

    public static final Struct<SoftLimitTrigger> struct =
        ProceduralStructGenerator.genEnum(SoftLimitTrigger.class);
  }

  private final EpilogueBackend logger;

  private DCMotorExt motor = null;
  private int numMotors = 0;
  private CurrentLimits currentLimits = CurrentLimits.base();
  private Time timeOverSupplyLimit = Seconds.of(0.0);
  private double sensorToMechanismRatio = 1.0;

  private Angle forwardSoftLimit = Radians.of(Double.POSITIVE_INFINITY);
  private Angle reverseSoftLimit = Radians.of(Double.NEGATIVE_INFINITY);

  private Optional<Output> output = Optional.empty();
  private boolean brakeMode = false;

  private Current lastCurrent = Amps.of(0.0);
  private Voltage lastVoltage = Volts.of(0.0);
  private MechanismState lastState = MechanismState.zero();

  public ShamMCX(String name) {
    logger = RuntimeLog.loggerFor("/ShamMCX/" + name).lazy();
  }

  public ShamMCX configureCurrentLimit(CurrentLimits currentLimit) {
    this.currentLimits = currentLimit;
    return this;
  }

  public ShamMCX configureSoftLimits(Angle forwardLimit, Angle reverseLimit) {
    this.forwardSoftLimit = forwardLimit;
    this.reverseSoftLimit = reverseLimit;
    return this;
  }

  public ShamMCX configSensorToMechanismRatio(double ratio) {
    this.sensorToMechanismRatio = ratio;
    return this;
  }

  private void logConfig() {
    logger.log("config/currentLimits", currentLimits, CurrentLimits.struct);
    logger.log("config/forwardSoftLimit", forwardSoftLimit);
    logger.log("config/reverseSoftLimit", reverseSoftLimit);
    logger.log("config/sensorToMechanismRatio", sensorToMechanismRatio);
  }

  @Override
  public void configureMotorModel(DCMotorExt motor) {
    this.motor = motor;
    this.numMotors = motor.numMotors;
  }

  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }

    logger.log("control/position", position);
    logger.log("control/velocity", velocity);
    logger.log("control/acceleration", RadiansPerSecondPerSecond.zero());
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlCurrent(controller, position, RadiansPerSecond.zero());
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }

    logger.log("control/position", Radians.zero());
    logger.log("control/velocity", velocity);
    logger.log("control/acceleration", acceleration);
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  public void controlCurrent(
      ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlCurrent(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  public void controlCurrent(Current amps) {

    logger.log("control/position", Radians.zero());
    logger.log("control/velocity", RadiansPerSecond.zero());
    logger.log("control/acceleration", RadiansPerSecondPerSecond.zero());
    output = Optional.of(Output.of(amps));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller,
      Angle position,
      AngularVelocity velocity) {
    if (controller == null || position == null) {
      output = Optional.empty();
      return;
    }

    logger.log("control/position", position);
    logger.log("control/velocity", velocity);
    logger.log("control/acceleration", RadiansPerSecondPerSecond.zero());
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, position, VU.of(velocity.baseUnitMagnitude())));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller, Angle position) {
    controlVoltage(controller, position, RadiansPerSecond.zero());
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity,
      AngularAcceleration acceleration) {
    if (controller == null || velocity == null) {
      output = Optional.empty();
      return;
    }

    logger.log("control/position", Radians.zero());
    logger.log("control/velocity", velocity);
    logger.log("control/acceleration", acceleration);
    output =
        Optional.of(
            new Output.ClosedLoopOutput<>(
                controller, velocity, AU.of(acceleration.baseUnitMagnitude())));
  }

  public void controlVoltage(
      ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller,
      AngularVelocity velocity) {
    controlVoltage(controller, velocity, RadiansPerSecondPerSecond.zero());
  }

  public void controlVoltage(Voltage volts) {

    logger.log("control/position", Radians.zero());
    logger.log("control/velocity", RadiansPerSecond.zero());
    logger.log("control/acceleration", RadiansPerSecondPerSecond.zero());
    output = Optional.of(Output.of(volts));
  }

  public Angle position() {
    return lastState.position();
  }

  public AngularVelocity velocity() {
    return lastState.velocity();
  }

  public AngularAcceleration acceleration() {
    return lastState.acceleration();
  }

  public Current current() {
    return lastCurrent;
  }

  public Voltage voltage() {
    return lastVoltage;
  }

  @Override
  public boolean brakeEnabled() {
    return brakeMode;
  }

  @Override
  public ControllerOutput run(Time dt, Voltage supply, MechanismState rawState) {
    logConfig();
    MechanismState state = rawState.div(sensorToMechanismRatio);
    lastState = state;
    return output
        .map(o -> o.run(dt, supply, state, logger.getNested("controlLoop")))
        .map(o -> logControllerOutput(o))
        .map(o -> softLimit(o, state))
        .map(o -> currentLimit(dt, o, state, supply))
        .orElseGet(() -> ControllerOutput.zero());
  }

  private ControllerOutput logControllerOutput(ControllerOutput output) {
    logger.log("output/brake", brakeMode);
    logger.log("output/mechanismState", lastState, MechanismState.struct);
    logger.log("output/type", output.type(), ControllerOutput.ControllerOutputTypes.struct);
    if (output instanceof VoltageOutput vo) {
      logger.log("output/magnitude", vo.voltage());
    } else if (output instanceof CurrentOutput co) {
      logger.log("output/magnitude", co.current());
    }
    return output;
  }

  private ControllerOutput softLimit(ControllerOutput requestedOutput, MechanismState state) {
    double direction = requestedOutput.signumMagnitude();
    Angle position = state.position();
    if (direction > 0 && position.in(Radians) > forwardSoftLimit.in(Radians)) {
      logger.log("softLimit/trigger", SoftLimitTrigger.FORWARD, SoftLimitTrigger.struct);
      return ControllerOutput.zero();
    } else if (direction < 0 && position.in(Radians) < reverseSoftLimit.in(Radians)) {
      logger.log("softLimit/trigger", SoftLimitTrigger.REVERSE, SoftLimitTrigger.struct);
      return ControllerOutput.zero();
    }
    logger.log("softLimit/trigger", SoftLimitTrigger.NONE, SoftLimitTrigger.struct);
    return requestedOutput;
  }

  private ControllerOutput currentLimit(
      Time dt, ControllerOutput requestedOutput, MechanismState state, Voltage supplyVoltage) {
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
    final CurrentLimits limits = currentLimits.times(numMotors);
    final AngularVelocity velocity = state.velocity();
    Voltage voltageInput;
    Current statorCurrent;
    Current supplyCurrent;

    if (requestedOutput instanceof VoltageOutput vo) {
      voltageInput = vo.voltage();
      statorCurrent = motor.getCurrent(velocity, voltageInput);
    } else {
      CurrentOutput co = (CurrentOutput) requestedOutput;
      statorCurrent = co.current();
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    }

    logger.log("output/rawStatorCurrent", statorCurrent);
    logger.log("output/rawVoltageInput", voltageInput);

    voltageInput = MeasureMath.clamp(voltageInput, supplyVoltage);
    supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);

    logger.log("output/rawSupplyCurrent", supplyCurrent);
    logger.log("output/rawSupplyVoltage", supplyVoltage);

    boolean overStatorLimit = statorCurrent.gt(limits.statorCurrentLimit);
    boolean overSupplyLimit = supplyCurrent.gt(limits.supplyCurrentLimit);
    boolean overLowerSupplyLimit =
        supplyCurrent.gt(limits.supplyCurrentLowerLimit)
            && timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime);

    logger.log("output/overStatorLimit", overStatorLimit);
    logger.log("output/overSupplyLimit", overSupplyLimit);
    logger.log("output/overLowerSupplyLimit", overLowerSupplyLimit);

    if (overStatorLimit) {
      statorCurrent = limits.statorCurrentLimit;
      voltageInput = motor.getVoltage(statorCurrent, velocity);
      supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);
    }

    if (overSupplyLimit) {
      timeOverSupplyLimit = timeOverSupplyLimit.plus(dt);
      Current supplyLimit = limits.supplyCurrentLimit;
      if (timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime)) {
        supplyLimit = limits.supplyCurrentLowerLimit;
      }
      statorCurrent =
          motor.getCurrent(velocity, voltageInput, supplyLimit, limits.statorCurrentLimit);
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    } else {
      timeOverSupplyLimit = Seconds.of(0.0);
    }

    logger.log("output/timeOverSupplyLimit", timeOverSupplyLimit);

    lastVoltage = voltageInput;
    lastCurrent = statorCurrent;

    logger.log("output/finalStatorCurrent", statorCurrent);
    logger.log("output/finalVoltageInput", voltageInput);

    if (requestedOutput instanceof VoltageOutput) {
      return ControllerOutput.of(voltageInput);
    } else {
      return ControllerOutput.of(statorCurrent);
    }
  }
}
