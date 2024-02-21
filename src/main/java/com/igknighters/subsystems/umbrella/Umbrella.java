package com.igknighters.subsystems.umbrella;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.subsystems.umbrella.intake.*;
import com.igknighters.subsystems.umbrella.shooter.*;
import com.igknighters.util.Tracer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Umbrella extends SubsystemBase {

    private final Intake intake;
    private final Shooter shooter;

    double stemLength = 0.66;
    double wristAngle = 0.97738438;
    double noteInitialVelocity = 33.528;

    private LoggedDashboardNumber speakerDistHorizontal = new LoggedDashboardNumber("speakerDistHorizontal", 0.0);
    private LoggedDashboardNumber speakerDistVertical = new LoggedDashboardNumber("speakerDistVertical", 0.0);

    public Umbrella() {
        if (RobotBase.isSimulation()) {
            intake = new IntakeSim();
            shooter = new ShooterSim();
        } else {
            intake = new IntakeReal();
            shooter = new ShooterReal();
        }
    }

    @Override
    public void periodic() {
        Tracer.startTrace("UmbrellaPeriodic");

        Tracer.traceFunc("IntakePeriodic", intake::periodic);
        Tracer.traceFunc("ShooterPeriodic", shooter::periodic);

        speakerDistHorizontal.get();
        Tracer.traceFunc("Aiming Math", () -> {
            double stemRads = StemSolvers.linearSolvePivotTheta(
                stemLength,
                wristAngle,
                speakerDistHorizontal.get(),
                speakerDistVertical.get()
            );

            SmartDashboard.putNumber("Stem Rads", stemRads);
            SmartDashboard.putNumber("Stem Deg", Math.toDegrees(stemRads));
        });

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
     * @return If the exit beam is broken
     */
    public boolean holdingGamepiece() {
        return intake.isExitBeamBroken();
    }

    /**
     * @return If the exit beam is not broken
     */
    public boolean notHoldingGamepiece() {
        return !intake.isExitBeamBroken();
    }

    /**
     * Runs the {@code Intake} at certain percent of its nominal voltage
     * 
     * @param percent The percent to run the intake at,
     *          1.0 is outtake, -1.0 is intake
     * @param force   If the intake should be forced to run
     */
    public void runIntakeAt(double percent, boolean force) {
        intake.setVoltageOut(12.0 * percent, force);
    }

    /**
     * Sets the intakes percent out
     * 
     * @param percent The percent to run the intake at,
     *          1.0 is outtake, -1.0 is intake
     */
    public void runIntakeAt(double percent) {
        intake.setVoltageOut(12.0 * percent);
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

    /**
     * Stops the {@code Intake} and {@code Shooter}
     */
    public void stopAll() {
        intake.setVoltageOut(0.0);
        shooter.setSpeed(0.0);
    }
}
