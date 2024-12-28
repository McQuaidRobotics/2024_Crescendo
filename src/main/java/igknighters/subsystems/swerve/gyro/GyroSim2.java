package igknighters.subsystems.swerve.gyro;

import igknighters.subsystems.swerve.odometryThread.SimSwerveOdometryThread;
import igknighters.util.logging.BootupLogger;
import sham.ShamGyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class GyroSim2 extends Gyro {

    private final Time dt;

    public GyroSim2(ShamGyro simGyro, SimSwerveOdometryThread odoThread) {
        this.dt = simGyro.getDt();
        simGyro.setUpdateConsumer((velo, accelVector) -> {
            super.yawRads += velo.times(dt).in(Radians);
            super.yawVelRadsPerSec = velo.in(RadiansPerSecond);
            super.accelX = accelVector.x().in(MetersPerSecondPerSecond);
            super.accelY = accelVector.y().in(MetersPerSecondPerSecond);
        });

        odoThread.addRotationSupplier(() -> Rotation2d.fromRadians(this.getYawRads()));
        odoThread.addAccelerationSupplier(() -> new double[] {this.accelX, this.accelY});

        BootupLogger.bootupLog("    Gyro initialized (sim)");
    }

    @Override
    public void setYawRads(double yawRads) {
        super.yawRads = yawRads;
    }
}
