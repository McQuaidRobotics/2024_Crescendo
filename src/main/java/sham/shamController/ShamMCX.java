package sham.shamController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import sham.ShamMotorController;
import sham.ShamMechanism.MechanismState;
import sham.ShamMotorController.ControllerOutput.CurrentOutput;
import sham.ShamMotorController.ControllerOutput.VoltageOutput;
import sham.shamController.UnitSafeControl.Feedforward;
import sham.shamController.UnitSafeControl.PDFeedback;
import sham.shamController.UnitSafeControl.TrapezoidProfile;
import sham.shamController.UnitSafeControl.TrapezoidProfile.State;
import sham.utils.DCMotor2;

public class ShamMCX implements ShamMotorController {
  private static final VelocityUnit<AngleUnit> VU = VelocityUnit.combine(Radians, Seconds);
  private static final AccelerationUnit<AngleUnit> AU = AccelerationUnit.combine(VU, Seconds);

  public static enum OutputType {
    VOLTAGE,
    CURRENT
  }

  public static enum OutputMode {
    VELOCITY,
    POSITION,
    OPEN_LOOP
  }

  public record Output(
      OutputMode mode,
      OutputType type,
      double value
  ) {
    public static Output voltage(double voltage) {
      return new Output(OutputMode.OPEN_LOOP, OutputType.VOLTAGE, voltage);
    }
  }

  public record CurrentLimits(
      Current statorCurrentLimit,
      Current supplyCurrentLimit,
      Current supplyCurrentLowerLimit,
      Time lowerLimitTriggerTime) {
    public static CurrentLimits base() {
      return new CurrentLimits(
          Amps.of(120.0),
          Amps.of(70.0),
          Amps.of(40.0),
          Seconds.of(1.0));
    }

    public CurrentLimits times(double factor) {
      return new CurrentLimits(
          statorCurrentLimit.times(factor),
          supplyCurrentLimit.times(factor),
          supplyCurrentLowerLimit.times(factor),
          lowerLimitTriggerTime
      );
    }
  }

  public record ClosedLoop<
      TYPE extends Unit,
      MODE extends Unit,
      DIM extends Unit
    >(
      PDFeedback<TYPE, MODE> feedback,
      Feedforward<TYPE, DIM> feedforward,
      Optional<TrapezoidProfile<MODE>> trapezoidProfile
    ) {}

  private final DCMotor2 motor;
  private final int numMotors;
  private CurrentLimits currentLimit = CurrentLimits.base();
  private Time timeOverSupplyLimit = Seconds.of(0.0);
  private double sensorToMechanismRatio = 1.0;

  private Angle forwardSoftLimit = Radians.of(Double.POSITIVE_INFINITY);
  private Angle reverseSoftLimit = Radians.of(Double.NEGATIVE_INFINITY);

  private ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> voltagePositionController;
  private ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> voltageVelocityController;
  private ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> currentPositionController;
  private ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> currentVelocityController;

  private Output output = Output.voltage(0.0);
  private boolean brakeMode = false;
  private MechanismState lastState = MechanismState.zero();

  /**
   * @param motor The DCMotor to use for the simulation
   * @param numMotors The number of motors in the mechanism
   */
  public ShamMCX(DCMotor motor, int numMotors) {
    this.numMotors = numMotors;
    this.motor = new DCMotor2(motor);
  }

  public ShamMCX configurePositionalVoltage(
    ClosedLoop<VoltageUnit, AngleUnit, AngleUnit> controller
  ) {
    this.voltagePositionController = controller;
    return this;
  }

  public ShamMCX configureVelocityVoltage(
    ClosedLoop<VoltageUnit, AngularVelocityUnit, AngleUnit> controller
  ) {
    this.voltageVelocityController = controller;
    return this;
  }

  public ShamMCX configurePositionalCurrent(
    ClosedLoop<CurrentUnit, AngleUnit, AngleUnit> controller
  ) {
    this.currentPositionController = controller;
    return this;
  }

  public ShamMCX configureVelocityCurrent(
    ClosedLoop<CurrentUnit, AngularVelocityUnit, AngleUnit> controller
  ) {
    this.currentVelocityController = controller;
    return this;
  }

  public ShamMCX configureCurrentLimit(CurrentLimits currentLimit) {
    this.currentLimit = currentLimit;
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

  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  public void setControl(OutputType outputType, AngularVelocity velo) {
    this.output = new Output(OutputMode.VELOCITY, outputType, velo.in(RadiansPerSecond));
  }

  public void setControl(OutputType outputType, Angle pos) {
    this.output = new Output(OutputMode.POSITION, outputType, pos.in(Radians));
  }

  public void setControl(Current amps) {
    this.output = new Output(OutputMode.OPEN_LOOP, OutputType.CURRENT, amps.in(Amps));
  }

  public void setControl(Voltage volts) {
    this.output = new Output(OutputMode.OPEN_LOOP, OutputType.VOLTAGE, volts.in(Volts));
  }

  public void setControl() {
    this.output = new Output(OutputMode.OPEN_LOOP, OutputType.VOLTAGE, 0.0);
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

  @Override
  public boolean brakeEnabled() {
    return brakeMode;
  }

  @Override
  public ControllerOutput run(Time dt, Voltage supply, MechanismState rawState) {
      MechanismState state = rawState.div(sensorToMechanismRatio);
      lastState = state;
      double dtSeconds = dt.in(Seconds);
      State<AngleUnit> profilePositionState = State.of(
        state.position(),
        state.velocity()
      );
      State<AngularVelocityUnit> profileVelocityState = State.of(
        state.velocity(),
        state.acceleration()
      );
      ControllerOutput controllerOutput = null;
      switch (this.output.type) {
        case VOLTAGE -> {
          switch (this.output.mode) {
            case OPEN_LOOP -> {
              controllerOutput = ControllerOutput.of(Volts.of(output.value));
            }
            case POSITION -> {
              Angle outputPosition = Radians.of(output.value);
              Voltage feedbackVoltage = (Voltage) voltagePositionController.feedback.calculate(
                state.position(),
                outputPosition
              );
              State<AngleUnit> profileState = voltagePositionController.trapezoidProfile.calculate(
                profilePositionState,
                State.of(outputPosition, RadiansPerSecond.zero()),
                dt
              );
              Acceleration<AngleUnit> accel = AU.of((
                state.velocity().baseUnitMagnitude() - profileState.slew().baseUnitMagnitude()
              ) / dtSeconds);
              Voltage feedforwardVoltage = (Voltage) voltagePositionController.feedforward.universalCalculate(
                profileState.value(),
                profileState.slew(),
                accel,
                state.position(),
                VU.of(state.velocity().in(RadiansPerSecond)),
                AU.of(state.acceleration().in(RadiansPerSecondPerSecond))
              );
              controllerOutput = ControllerOutput.of(feedforwardVoltage.plus(feedbackVoltage));
            }
            case VELOCITY -> {
              AngularVelocity outputVelocity = RadiansPerSecond.of(output.value);
              Voltage feedbackVoltage = (Voltage) voltageVelocityController.feedback.calculate(
                state.velocity(),
                outputVelocity
              );
              State<AngularVelocityUnit> profileState = voltageVelocityController.trapezoidProfile.calculate(
                profileVelocityState,
                State.of(outputVelocity, RadiansPerSecondPerSecond.zero()),
                dt
              );
              Voltage feedforwardVoltage = (Voltage) voltagePositionController.feedforward.universalCalculate(
                state.position(),
                VU.of(profileState.value().in(RadiansPerSecond)),
                AU.of(profileState.slew().baseUnitMagnitude()),
                state.position(),
                VU.of(state.velocity().in(RadiansPerSecond)),
                AU.of(state.acceleration().in(RadiansPerSecondPerSecond))
              );
              controllerOutput = ControllerOutput.of(feedforwardVoltage.plus(feedbackVoltage));
            }
          }
        }
        case CURRENT -> {
          switch (this.output.mode) {
            case OPEN_LOOP -> {
              controllerOutput = ControllerOutput.of(Volts.of(output.value));
            }
            case POSITION -> {
              Angle outputPosition = Radians.of(output.value);
              Current feedbackCurrent = (Current) currentPositionController.feedback.calculate(
                state.position(),
                outputPosition
              );
              State<AngleUnit> profileState = currentPositionController.trapezoidProfile.calculate(
                profilePositionState,
                State.of(outputPosition, RadiansPerSecond.zero()),
                dt
              );
              Acceleration<AngleUnit> accel = AU.of((
                state.velocity().baseUnitMagnitude() - profileState.slew().baseUnitMagnitude()
              ) / dtSeconds);
              Current feedforwardCurrent = (Current) currentPositionController.feedforward.universalCalculate(
                profileState.value(),
                profileState.slew(),
                accel,
                state.position(),
                VU.of(state.velocity().in(RadiansPerSecond)),
                AU.of(state.acceleration().in(RadiansPerSecondPerSecond))
              );
              controllerOutput = ControllerOutput.of(feedforwardCurrent.plus(feedbackCurrent));
            }
            case VELOCITY -> {
              AngularVelocity outputVelocity = RadiansPerSecond.of(output.value);
              Current feedbackCurrent = (Current) currentVelocityController.feedback.calculate(
                state.velocity(),
                outputVelocity
              );
              State<AngularVelocityUnit> profileState = currentVelocityController.trapezoidProfile.calculate(
                profileVelocityState,
                State.of(outputVelocity, RadiansPerSecondPerSecond.zero()),
                dt
              );
              Current feedforwardCurrent = (Current) currentPositionController.feedforward.universalCalculate(
                state.position(),
                VU.of(profileState.value().in(RadiansPerSecond)),
                AU.of(profileState.slew().baseUnitMagnitude()),
                state.position(),
                VU.of(state.velocity().in(RadiansPerSecond)),
                AU.of(state.acceleration().in(RadiansPerSecondPerSecond))
              );
              controllerOutput = ControllerOutput.of(feedforwardCurrent.plus(feedbackCurrent));
            }
          }
        }
      }
      controllerOutput = softLimit(controllerOutput, state);
      return currentLimit(dt, controllerOutput, state);
  }

  private ControllerOutput softLimit(ControllerOutput requestedOutput, MechanismState state) {
    double direction = requestedOutput.signumMagnitude();
    Angle position = state.position();
    if (direction > 0 && position.in(Radians) > forwardSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    } else if (direction < 0 && position.in(Radians) < reverseSoftLimit.in(Radians)) {
      return ControllerOutput.zero();
    }
    return requestedOutput;
  }


  private ControllerOutput currentLimit(Time dt, ControllerOutput requestedOutput, MechanismState state) {
    // https://file.tavsys.net/control/controls-engineering-in-frc.pdf (sec 12.1.3)
    final CurrentLimits limits = currentLimit.times(numMotors);
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
    supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);

    if (statorCurrent.gt(limits.statorCurrentLimit)) {
      statorCurrent = limits.statorCurrentLimit;
      voltageInput = motor.getVoltage(statorCurrent, velocity);
      supplyCurrent = motor.getSupplyCurrent(velocity, voltageInput, statorCurrent);
    }

    if (supplyCurrent.gt(limits.supplyCurrentLimit)) {
      timeOverSupplyLimit = timeOverSupplyLimit.plus(dt);
      Current supplyLimit = limits.supplyCurrentLimit;
      if (timeOverSupplyLimit.gt(limits.lowerLimitTriggerTime)) {
        supplyLimit = limits.supplyCurrentLowerLimit;
      }
      statorCurrent = motor.getCurrent(velocity, voltageInput, supplyLimit, limits.statorCurrentLimit);
      voltageInput = motor.getVoltage(statorCurrent, velocity);
    } else {
      timeOverSupplyLimit = Seconds.of(0.0);
    }

    if (requestedOutput instanceof VoltageOutput) {
      return ControllerOutput.of(voltageInput);
    } else {
      return ControllerOutput.of(statorCurrent);
    }
  }
}
