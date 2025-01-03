package sham.utils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.proto.DCMotorProto;
import edu.wpi.first.math.system.plant.struct.DCMotorStruct;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import sham.utils.mathutils.MeasureMath;

public class DCMotor2 extends DCMotor {

  public static final DCMotorProto proto = new DCMotorProto();
  public static final DCMotorStruct struct = new DCMotorStruct();

  public DCMotor2(
      double nominalVoltageVolts,
      double stallTorqueNewtonMeters,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRadPerSec,
      int numMotors) {
    super(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec,
        numMotors);
  }

  public DCMotor2(DCMotor motor) {
    super(
      motor.nominalVoltageVolts,
      motor.stallTorqueNewtonMeters,
      motor.stallCurrentAmps,
      motor.freeCurrentAmps,
      motor.freeSpeedRadPerSec,
      1
    );
  }

  /**
   * Calculate current drawn by motor with given speed and input voltage.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInputVolts The voltage being applied to the motor.
   * @return The estimated current.
   */
  public Current getCurrent(AngularVelocity speed, Voltage voltageInput) {
    return Amps.of(super.getCurrent(speed.in(RadiansPerSecond), voltageInput.in(Volts)));
  }

  /**
   * Calculate current drawn by motor for a given torque.
   *
   * @param torque The torque produced by the motor.
   * @return The current drawn by the motor.
   */
  public Current getCurrent(Torque torque) {
    return Amps.of(super.getCurrent(torque.in(NewtonMeters)));
  }

  /**
   * Calculate torque produced by the motor with a given current.
   *
   * @param current The current drawn by the stator.
   * @return The torque output.
   */
  public Torque getTorque(Current current) {
    return NewtonMeters.of(super.getTorque(current.in(Amps)));
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param torque The torque produced by the motor.
   * @param speed The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public Voltage getVoltage(Torque torque, AngularVelocity speed) {
    return Volts.of(super.getVoltage(torque.in(NewtonMeters), speed.in(RadiansPerSecond)));
  }

  /**
   * Calculate the voltage provided to the motor for a given torque and angular velocity.
   *
   * @param current The current drawn by the stator.
   * @param speed The current angular velocity of the motor.
   * @return The voltage of the motor.
   */
  public Voltage getVoltage(Current current, AngularVelocity speed) {
    return Volts.of(super.getVoltage(super.getTorque(current.in(Amps)), speed.in(RadiansPerSecond)));
  }

  /**
   * Calculates the angular speed produced by the motor at a given torque and input voltage.
   *
   * @param torque The torque produced by the motor.
   * @param voltageInput The voltage applied to the motor.
   * @return The angular speed of the motor.
   */
  public AngularVelocity getSpeed(Torque torque, Voltage voltageInput) {
    return RadiansPerSecond.of(super.getSpeed(torque.in(NewtonMeters), voltageInput.in(Volts)));
  }

  /**
   * Calculates the percent output of the motor at a given speed.
   *
   * @param speed The current angular velocity of the motor.
   * @return The percent output of the motor.
   */
  public double getSpeedPercent(AngularVelocity speed) {
    return speed.in(RadiansPerSecond) / freeSpeedRadPerSec;
  }

  /**
   * Calculates the free current of the motor at a given speed.
   *
   * @param speed The current angular velocity of the motor.
   * @return The free current of the motor.
   */
  public Current getFreeCurrent(AngularVelocity speed) {
    return Amps.of(freeCurrentAmps * getSpeedPercent(speed));
  }

  /**
   * Calculates the maximum power output of a motor at
   * a given applied voltage and supply current limit.
   *
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @return The maximum power output of the motor.
   */
  public Power getMaxPower(Voltage voltageInput, Current supplyLimit) {
    return voltageInput.times(supplyLimit);
  }

  /**
   * Calculates the maximum stator current of the motor at a given speed,
   * input voltage and current limits.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @param statorLimit The current limit of the stator.
   * @return The maximum stator current of the motor.
   */
  public Current getMaxStator(AngularVelocity speed, Voltage voltageInput, Current supplyLimit, Current statorLimit) {
    // Theres a lot of unit coercion happening here that the unit lib doesn't natively support.
    // Lower the math to type unsafe double math.
    double vIn = voltageInput.in(Volts);
    double vOut = voltageInput.times(getSpeedPercent(speed)).in(Volts);
    double maxP = getMaxPower(voltageInput, supplyLimit).in(Watts);
    double freeCurrent = getFreeCurrent(speed).in(Amps);

    double xSqr = (vOut * vOut) - (4.0 * rOhms * (-maxP + (vIn * freeCurrent)));

    if (xSqr < 0) {
      return MeasureMath.abs(statorLimit);
    }

    double y = Math.abs(-vOut + Math.sqrt(xSqr) / (2.0 * rOhms));

    return Amps.of(y);
  }

  /**
   * Calculates the stall current of the motor at a given speed.
   *
   * @param speed The current angular velocity of the motor.
   * @return The stall current of the motor.
   */
  public Current getStallCurrent(AngularVelocity speed) {
    return Amps.of((1.0 - getSpeedPercent(speed)) * stallCurrentAmps);
  }

  /**
   * Calculates the stator current of the motor at a given speed, input voltage,
   * supply current limit and stator current limit.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param supplyLimit The current limit of the power supply.
   * @param statorLimit The current limit of the stator.
   * @return The stator current of the motor.
   */
  public Current getCurrent(AngularVelocity speed, Voltage voltageInput, Current supplyLimit, Current statorLimit) {
    Current maxStator = getMaxStator(speed, voltageInput, supplyLimit, statorLimit).minus(getFreeCurrent(speed));
    Current stall = getStallCurrent(speed);

    Current stallOrLimit = MeasureMath.min(stall, statorLimit);
    Current stallOrLimitAboveZero = MeasureMath.max(stallOrLimit, Amps.of(0.0));
    return MeasureMath.min(stallOrLimitAboveZero, maxStator);
  }

  /**
   * Calculates the supply current of the motor at a given speed, input voltage and stator current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param current The current drawn by the motor.
   * @return The supply current of the motor.
   */
  public Current getSupplyCurrent(AngularVelocity speed, Voltage voltageInput, Current current) {
    Power statorPower = voltageInput.times(current).times(getSpeedPercent(speed));
    Power losses = getLosses(speed, voltageInput, current);
    return statorPower.plus(losses).div(voltageInput);
  }

  /**
   * Calculates the output power of the motor at a given speed and torque.
   *
   * @param speed The current angular velocity of the motor.
   * @param torque The torque produced by the motor.
   * @return The output power of the motor.
   */
  public Power getOutputPower(AngularVelocity speed, Torque torque) {
    return Watts.of(speed.in(RadiansPerSecond) * torque.in(NewtonMeters));
  }

  /**
   * Calculates the losses of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param current The current drawn by the motor.
   * @return The losses of the motor.
   */
  public Power getOutputPower(AngularVelocity speed, Current current) {
    Torque torque = getTorque(current);
    return Watts.of(speed.in(RadiansPerSecond) * torque.in(NewtonMeters));
  }

  /**
   * Calculates the losses of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param current The current drawn by the motor.
   * @return The losses of the motor.
   */
  public Power getLosses(AngularVelocity speed, Voltage voltageInput, Current current) {
    Power passiveLoss = getFreeCurrent(speed).times(voltageInput);
    Power resistiveLoss = Watts.of(rOhms * Math.pow(current.in(Amps), 2.0));
    return passiveLoss.plus(resistiveLoss);
  }

  /**
   * Calculates the efficiency of the motor at a given speed, input voltage and current.
   *
   * @param speed The current angular velocity of the motor.
   * @param voltageInput The voltage applied to the motor.
   * @param current The current drawn by the motor.
   * @return The efficiency of the motor.
   */
  public double getEfficiency(AngularVelocity speed, Voltage voltageInput, Current current) {
    Power output = getOutputPower(speed, current);
    Power losses = getLosses(speed, voltageInput, current);
    return output.in(Watts) / (output.in(Watts) + losses.in(Watts));
  }
}
