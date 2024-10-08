package com.igknighters.subsystems.umbrella;

import java.util.List;
import java.util.function.Consumer;

import com.igknighters.Localizer;
import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import com.igknighters.subsystems.umbrella.intake.*;
import com.igknighters.subsystems.umbrella.shooter.*;
import com.igknighters.util.GamepieceSimulator;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.logging.Tracer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * 
 */
public class Umbrella implements LockFullSubsystem {
    private final Intake intake;
    private final Shooter shooter;

    public Umbrella() {
        if (Robot.isSimulation()) {
            intake = new IntakeSim();
            shooter = new ShooterDisabled();
        } else {
            if (Robot.isSunlight()) {
                intake = new IntakeRealSingleCurrent();
            } else {
                intake = new IntakeRealSingle();
            }
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
     * @return If the {@code Intake} is running inwards
     */
    public boolean isIntaking() {
        return intake.getVoltageOut() < 0.0;
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
     * @param rpm The speed to spin up to in rotations per minute
     */
    public void spinupShooterToRPM(double rpm) {
        shooter.setSpeed(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /**
     * Stops the {@code Intake} and {@code Shooter}
     */
    public void stopAll() {
        intake.setVoltageOut(0.0);
        shooter.setSpeed(0.0);
    }

    /**
     * Sets up the simulation for intaking notes in simulation
     * 
     * @param localizer The {@link Localizer} to use for the simulation
     */
    public void setupSimNoteDetection(Localizer localizer) {
        if (!Robot.isSimulation() || !(intake instanceof IntakeSim)) {
            return;
        }

        IntakeSim intakeSim = (IntakeSim) intake;
        var s = localizer.namedPositionsSender();
        Consumer<Pose2d[]> noteSender = poses -> s.send(new Localizer.NamedPositions("Notes", poses));

        GamepieceSimulator.setupGamepieceAcquisitionSim(
            localizer::pose,
            this::isIntaking,
            notes -> {
                Pose2d[] notePoses = notes.stream().map(p -> new Pose2d(p, GeomUtil.ROTATION2D_ZERO)).toArray(Pose2d[]::new);
                noteSender.accept(notePoses);
            },
            List.of(
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(24.0),
                        Units.inchesToMeters(0.0)
                    ),
                    GeomUtil.ROTATION2D_ZERO
                ),
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(24.0),
                        Units.inchesToMeters(-7.0)
                    ),
                    GeomUtil.ROTATION2D_ZERO
                ),
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(24.0),
                        Units.inchesToMeters(7.0)
                    ),
                    GeomUtil.ROTATION2D_ZERO
                )
            )
        ).onTrue(
            Commands.runOnce(() -> intakeSim.setExitBeam(true))
                .withName("SetExitBeamTrue")
        );
    }
}
