package org.ironmaple;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ironmaple.SimArena.SimulationTiming;
import org.ironmaple.configs.GyroConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.MapleCommonMath;

/**
 * Simulation for a IMU module used as gyro.
 *
 * <p>The Simulation is basically an indefinite integral of the angular velocity during each simulation sub ticks. Above
 * that, it also musicales the measurement inaccuracy of the gyro, drifting in no-motion and drifting due to impacts.
 */
public class SimGyro {
    /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
    private static final double ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING = 500;
    /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
    private static final double DRIFT_DUE_TO_IMPACT_COEFFICIENT = Math.toRadians(1);

    private final SimulationTiming timing;

    private final double averageDriftingIn30SecsMotionlessDeg, velocityMeasurementStandardDeviationPercent;

    private Rotation2d gyroReading;
    private double measuredAngularVelocityRadPerSec, previousAngularVelocityRadPerSec;
    private final Queue<Rotation2d> cachedRotations;

    /**
     *
     *
     * <h2>Creates a Gyro Simulation.</h2>
     *
     * @param averageDriftingIn30SecsMotionlessDeg the average amount of drift, in degrees, the gyro experiences
     *     if it remains motionless for 30 seconds on a vibrating platform. This value can often be found in the user
     *     manual.
     * @param velocityMeasurementStandardDeviationPercent the standard deviation of the velocity measurement,
     *     typically around 0.05
     */
    public SimGyro(SimulationTiming timing, GyroConfig gyroConfig) {
        this.timing = timing;
        this.averageDriftingIn30SecsMotionlessDeg = gyroConfig.averageDriftingIn30SecsMotionlessDeg;
        this.velocityMeasurementStandardDeviationPercent = gyroConfig.velocityMeasurementStandardDeviationPercent;

        gyroReading = new Rotation2d();
        this.previousAngularVelocityRadPerSec = this.measuredAngularVelocityRadPerSec = 0;
        this.cachedRotations = new ConcurrentLinkedQueue<>();
        for (int i = 0; i < timing.ticksPerPeriod; i++) cachedRotations.offer(gyroReading);

        RuntimeLog.debug("Created a swerve module simulation");
    }

    /**
     *
     *
     * <h2>Calibrates the Gyro to a Given Rotation.</h2>
     *
     * <p>This method sets the current rotation of the gyro, similar to <code>Pigeon2().setYaw()
     * </code>.
     *
     * <p>After setting the rotation, the gyro will continue to estimate the rotation by integrating the angular
     * velocity, adding it to the specified rotation.
     *
     * @param currentRotation the current rotation of the robot, represented as a {@link Rotation2d}
     */
    public void setRotation(Rotation2d currentRotation) {
        this.gyroReading = currentRotation;
    }

    /**
     *
     *
     * <h2>Obtains the Estimated Rotation of the Gyro.</h2>
     *
     * <p>This method returns the estimated rotation of the gyro, which includes measurement errors due to drifting and
     * other factors.
     *
     * @return the current reading of the gyro, represented as a {@link Rotation2d}
     */
    public Rotation2d getGyroReading() {
        return gyroReading;
    }

    /**
     *
     *
     * <h2>Gets the Measured Angular Velocity of the Gyro.</h2>
     *
     * <p>This method returns the angular velocity measured by the gyro, in radians per second.
     *
     * <p>The measurement includes random errors based on the configured settings of the gyro.
     *
     * @return the measured angular velocity in radians per second
     */
    public double getMeasuredAngularVelocityRadPerSec() {
        return measuredAngularVelocityRadPerSec;
    }

    /**
     * gyro readings for <a
     * href="https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/update-frequency-impact.html">high-frequency
     * odometers</a>.
     *
     * @return the readings of the gyro during the last 5 simulation sub ticks
     */
    public Rotation2d[] getCachedGyroReadings() {
        return cachedRotations.toArray(Rotation2d[]::new);
    }

    /**
     *
     *
     * <h2>Updates the Gyro Simulation for Each Sub-Tick.</h2>
     *
     * <p>This method updates the gyro simulation and should be called during every sub-tick of the simulation.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, usually obtained from
     *     {@link SimDriveTrain#getAngularVelocity()}
     */
    public void updateSimulationSubTick(double actualAngularVelocityRadPerSec) {
        final Rotation2d driftingDueToImpact = getDriftingDueToImpact(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(driftingDueToImpact);

        final Rotation2d dTheta = getGyroDTheta(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(dTheta);

        final Rotation2d noMotionDrifting = getNoMotionDrifting();
        gyroReading = gyroReading.plus(noMotionDrifting);

        cachedRotations.poll();
        cachedRotations.offer(gyroReading);
    }

    /**
     *
     *
     * <h2>Simulates IMU Drifting Due to Robot Impacts.</h2>
     *
     * <p>This method generates a random amount of drifting for the IMU if the instantaneous angular acceleration
     * exceeds a threshold, simulating the effects of impacts on the robot.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, used to determine if an
     *     impact is detected
     * @return the amount of drifting the IMU will experience if an impact is detected, or <code>
     *     Rotation2d.fromRadians(0)</code> if no impact is detected
     */
    private Rotation2d getDriftingDueToImpact(double actualAngularVelocityRadPerSec) {
        final double
                angularAccelerationRadPerSecSq =
                        (actualAngularVelocityRadPerSec - previousAngularVelocityRadPerSec) / timing.dt,
                driftingDueToImpactDegAbsVal =
                        Math.abs(angularAccelerationRadPerSecSq) > ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING
                                ? Math.abs(angularAccelerationRadPerSecSq)
                                        / ANGULAR_ACCELERATION_THRESHOLD_START_DRIFTING
                                        * DRIFT_DUE_TO_IMPACT_COEFFICIENT
                                : 0,
                driftingDueToImpactDeg = Math.copySign(driftingDueToImpactDegAbsVal, -angularAccelerationRadPerSecSq);

        previousAngularVelocityRadPerSec = actualAngularVelocityRadPerSec;

        return Rotation2d.fromRadians(driftingDueToImpactDeg);
    }

    /**
     *
     *
     * <h2>Gets the Measured ΔTheta of the Gyro.</h2>
     *
     * <p>This method simulates the change in the robot's angle (ΔTheta) since the last sub-tick, as measured by the
     * gyro.
     *
     * <p>The measurement includes random errors based on the configuration of the gyro.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, used to calculate the
     *     ΔTheta
     * @return the measured ΔTheta, including any measurement errors
     */
    private Rotation2d getGyroDTheta(double actualAngularVelocityRadPerSec) {
        this.measuredAngularVelocityRadPerSec = MapleCommonMath.generateRandomNormal(
                actualAngularVelocityRadPerSec,
                velocityMeasurementStandardDeviationPercent * Math.abs(actualAngularVelocityRadPerSec));
        return Rotation2d.fromRadians(measuredAngularVelocityRadPerSec * timing.dt);
    }

    /**
     *
     *
     * <h2>Generates the No-Motion Gyro Drifting.</h2>
     *
     * <p>This method simulates the minor drifting of the gyro that occurs regardless of whether the robot is moving or
     * not.
     *
     * @return the amount of drifting generated while the robot is not moving
     */
    private Rotation2d getNoMotionDrifting() {
        final double
                AVERAGE_DRIFTING_1_PERIOD =
                        this.averageDriftingIn30SecsMotionlessDeg / 30 * timing.dt,
                driftingInThisPeriod = MapleCommonMath.generateRandomNormal(0, AVERAGE_DRIFTING_1_PERIOD);

        return Rotation2d.fromDegrees(driftingInThisPeriod);
    }
}
