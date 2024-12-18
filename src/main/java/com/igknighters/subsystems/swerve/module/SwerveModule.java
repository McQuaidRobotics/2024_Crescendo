package com.igknighters.subsystems.swerve.module;

import com.igknighters.subsystems.Component;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;

public abstract class SwerveModule extends Component {
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

    @Override
    public String getOverrideName() {
        return "SwerveModule";
    }

    /**
     * @param desiredState The state that the module should assume, angle and
     *                     velocity.
     * @param isOpenLoop   Whether the module speed assumed should be reached via
     *                     open or closed loop control.
     */
    public abstract void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

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