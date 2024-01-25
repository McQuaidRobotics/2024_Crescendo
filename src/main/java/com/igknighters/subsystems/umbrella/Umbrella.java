package com.igknighters.subsystems.umbrella;

import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.umbrella.intake.*;
import com.igknighters.subsystems.umbrella.shooter.*;
import com.igknighters.util.Tracer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Umbrella extends SubsystemBase {

    private final Intake intake;
    private final Shooter shooter;

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

        Tracer.endTrace();
    }

    /**
     * @return If the {@code Shooter} is at the target speed
     * 
     * @apiNote This uses the default tolerance of {@link kShooter#DEFAULT_TOLERANCE}
     */
    public boolean isShooterAtSpeed() {
        return isShooterAtSpeed(kShooter.DEFAULT_TOLERANCE);
    }

    /**
     * @param tolerance The tolerance to use when checking if the {@code Shooter} is at speed
     * @return If the {@code Shooter} is at the target speed
     * 
     * @apiNote Tolerance is a percentage of the target speed to allow for error,
     *         so a tolerance of 0.1 would allow for a 10% error
     */
    public boolean isShooterAtSpeed(double tolerance) {
        return Math.abs(shooter.getSpeed() - shooter.getTargetSpeed()) < tolerance;
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
        intake.turnIntakeMeters(meters);
    }

    public void spinupShooterToSurfaceSpeed(double surfaceMPS) {
        // shooter.setSpeed(radPerSec);
    }

    public void spinupShooterToRotSpeed() {}
}
