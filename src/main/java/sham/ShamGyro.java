package sham;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.procstruct.ProceduralStructGenerator;
import monologue.procstruct.ProceduralStructGenerator.IgnoreStructField;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BiConsumer;

import sham.ShamArena.ShamEnvTiming;
import sham.configs.ShamGyroConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.MapleCommonMath;
import sham.utils.mathutils.MeasureMath;

/**
 * Simulation for a IMU module used as gyro.
 *
 * <p>The Simulation is basically an indefinite integral of the angular velocity during each simulation sub ticks. Above
 * that, it also musicales the measurement inaccuracy of the gyro, drifting in no-motion and drifting due to impacts.
 */
public class ShamGyro implements StructSerializable {
    /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
    private static final AngularAcceleration START_DRIFTING = RadiansPerSecondPerSecond.of(500);
    /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
    private static final Angle DRIFT_DUE_TO_IMPACT_COEFFICIENT = Radians.of(1);

    @IgnoreStructField
    private final ShamEnvTiming timing;
    @IgnoreStructField
    private BiConsumer<Time, AngularVelocity> yawVeloConsumer;

    private final double veloStdDev;

    private final AngularVelocity averageDriftingMotionless;

    private AngularVelocity lastAngularVelocity = RadiansPerSecond.of(0);

    /**
     *
     *
     * <h2>Creates a Gyro Simulation.</h2>
     *
     * @param averageDriftingIn30SecsMotionlessDeg the average amount of drift, in degrees, the gyro experiences
     *     if it remains motionless for 30 seconds on a vibrating platform. This value can often be found in the user
     *     manual.
     * @param veloStdDev the standard deviation of the velocity measurement,
     *     typically around 0.05
     */
    public ShamGyro(ShamEnvTiming timing, ShamGyroConfig gyroConfig) {
        this.timing = timing;
        this.averageDriftingMotionless = Degrees.of(gyroConfig.averageDriftingIn30SecsMotionlessDeg)
                .div(Seconds.of(30.0));
        this.veloStdDev = gyroConfig.velocityMeasurementStandardDeviationPercent;

        RuntimeLog.debug("Created a swerve module simulation");
    }

    public void setYawVeloConsumer(BiConsumer<Time, AngularVelocity> yawVeloConsumer) {
        this.yawVeloConsumer = yawVeloConsumer;
    }

    /**
     *
     *
     * <h2>Updates the Gyro Simulation for Each Sub-Tick.</h2>
     *
     * <p>This method updates the gyro simulation and should be called during every sub-tick of the simulation.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, usually obtained from
     *     {@link ShamDriveTrain#getAngularVelocity()}
     */
    public void updateSimulationSubTick(AngularVelocity actualAngularVelocity) {
        AngularVelocity dTheta = actualAngularVelocity
                .plus(averageDriftingMotionless)
                .plus(getDriftingDueToImpact(actualAngularVelocity))
                .plus(actualAngularVelocity
                        .times(MapleCommonMath.generateRandomNormal(0.0, veloStdDev))
                );

        lastAngularVelocity = dTheta;

        if (yawVeloConsumer != null) {
            yawVeloConsumer.accept(timing.dt(), lastAngularVelocity);
        }
    }

    private AngularVelocity getDriftingDueToImpact(AngularVelocity actualAngularVelocity) {
        AngularAcceleration angularAcceleration = actualAngularVelocity.minus(lastAngularVelocity).div(timing.dt());
        if (MeasureMath.abs(angularAcceleration).gt(START_DRIFTING)) {
            return DRIFT_DUE_TO_IMPACT_COEFFICIENT
                    .times(MeasureMath.signum(angularAcceleration))
                    .times(angularAcceleration.div(START_DRIFTING))
                    .div(timing.dt());
        } else {
            return RadiansPerSecond.of(0);
        }
    }

    public static final Struct<ShamGyro> struct = ProceduralStructGenerator.genObjectNoUnpack(ShamGyro.class);
}
