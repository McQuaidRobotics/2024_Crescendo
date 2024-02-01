package com.igknighters.subsystems.umbrella;

import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.intake.*;
import com.igknighters.subsystems.umbrella.shooter.*;
import com.igknighters.util.Tracer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Umbrella extends SubsystemBase {

    private final Intake intake;
    private final Shooter shooter;

    public Umbrella() {
        if (RobotBase.isSimulation()) {
            intake = new IntakeSim();
            shooter = new ShooterSim();
        } else {
            intake = new IntakeSim();
            shooter = new ShooterReal();
        }
    }

    @Override
    public void periodic() {
        Tracer.startTrace("UmbrellaPeriodic");

        Tracer.traceFunc("IntakePeriodic", intake::periodic);
        Tracer.traceFunc("ShooterPeriodic", shooter::periodic);

        Tracer.endTrace();
    }

    /**
     * @return If the {@code Shooter} is at the target speed
     * 
     * @apiNote This uses the default tolerance of
     *          {@link kShooter#DEFAULT_TOLERANCE}
     */
    public boolean isShooterAtSpeed() {
        return isShooterAtSpeed(kShooter.DEFAULT_TOLERANCE);
    }

    /**
     * @param tolerance The tolerance to use when checking if the {@code Shooter} is
     *                  at speed
     * @return If the {@code Shooter} is at the target speed
     * 
     * @apiNote Tolerance is a percentage of the target speed to allow for error,
     *          so a tolerance of 0.1 would allow for a 10% error
     */
    public boolean isShooterAtSpeed(double tolerance) {
        return (Math.abs(shooter.getSpeed() - shooter.getTargetSpeed()) / shooter.getTargetSpeed()) < tolerance;
    }

    /**
     * @return The speed of the {@code Shooter} in radians per second
     */
    public double getShooterSpeed() {
        return shooter.getSpeed();
    }

    /**
     * @return The target speed of the {@code Shooter} in radians per second
     */
    public double getShooterTargetSpeed() {
        return shooter.getTargetSpeed();
    }

    /**
     * @return If the entrance beam is broken
     */
    public boolean isEntranceBeamBroken() {
        return intake.isEntranceBeamBroken();
    }

    /**
     * @return If the exit beam is broken
     */
    public boolean isExitBeamBroken() {
        return intake.isExitBeamBroken();
    }

    /**
     * Sets the intakes percent out
     */
    public void runIntakeAt(double percent) {
        intake.setVoltageOut(12.0 * percent);
    }

    public void turnIntakeBy(double meters) {
        // TODO
        // var wheelCircum = kIntake.WHEEL_DIAMETER * Math.PI;
        // intake.turnIntakeMeters(Units.rotationsToRadians(surfaceMPS / wheelCircum));
    }

    /**
     * Spins up the {@code Shooter} to a certain speed
     * 
     * @param radiansPerSecond The speed to spin up to in radians per second
     */
    public void spinupShooter(double radiansPerSecond) {
        shooter.setSpeed(radiansPerSecond);
    }

    /**
     * Spins up the {@code Shooter} to a certain speed
     * 
     * @param surfaceMPS The speed to spin up to in meters per second
     */
    public void spinupShooterToSurfaceSpeed(double surfaceMPS) {
        var wheelCircum = kShooter.WHEEL_DIAMETER * Math.PI;
        shooter.setSpeed(Units.rotationsToRadians(surfaceMPS / wheelCircum));
    }

    /**
     * Spins up the {@code Shooter} to a certain speed
     * 
     * @param RPM The speed to spin up to in rotations per minute
     */
    public void spinupShooterToRPM(double RPM) {
        shooter.setSpeed(Units.rotationsPerMinuteToRadiansPerSecond(RPM));
    }

    public Command run(Runnable action) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }
}
