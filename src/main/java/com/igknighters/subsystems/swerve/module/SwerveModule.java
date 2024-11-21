package com.igknighters.subsystems.swerve.module;

import com.igknighters.subsystems.Component;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import monologue.Annotations.Log;

public abstract class SwerveModule extends Component {
    public static final class AdvancedSwerveModuleState extends SwerveModuleState {
        public double steerVelocityFF;
        public double driveAccelerationFF;

        public AdvancedSwerveModuleState(double speedMetersPerSecond, Rotation2d angle, double steerVelocityFF,
                double driveAccelerationFF) {
            super(speedMetersPerSecond, angle);
            this.steerVelocityFF = steerVelocityFF;
            this.driveAccelerationFF = driveAccelerationFF;
        }

        //todo: implement custom struct
        public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

        public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
            return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0, 0.0);
        }
    }

    @Log
    public double driveVeloMPS = 0.0;
    @Log
    public double targetDriveVeloMPS = 0.0;
    @Log
    public double drivePositionMeters = 0.0;
    @Log
    public double driveVolts = 0.0;
    @Log
    public double driveAmps = 0.0;
    @Log
    public double angleVeloRadPS = 0.0;
    @Log
    public double angleAbsoluteRads = 0.0;
    @Log
    public double targetAngleAbsoluteRads = 0.0;
    @Log
    public double angleVolts = 0.0;
    @Log
    public double angleAmps = 0.0;

    public final String name;

    protected SwerveModule(String name) {
        this.name = name;
    }

    /**
     * @param desiredState The state that the module should assume, angle and
     *                     velocity.
     * @param isOpenLoop   Whether the module speed assumed should be reached via
     *                     open or closed loop control.
     */
    public abstract void setDesiredState(AdvancedSwerveModuleState desiredState);

    /**
     * @return The velocity/angle of the module.
     */
    public abstract SwerveModuleState getCurrentState();

    public abstract SwerveModulePosition getCurrentPosition();

    /**
     * @return Returns the module's assigned number in the {@link Swerve#swerveMods}
     *         array.
     */
    public abstract int getModuleNumber();

    public abstract void setVoltageOut(double volts, Rotation2d angle);
}