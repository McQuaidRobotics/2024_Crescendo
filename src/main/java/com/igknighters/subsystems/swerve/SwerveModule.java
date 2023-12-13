package com.igknighters.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public interface SwerveModule {
    public static class SwerveModuleInputs implements LoggableInputs {
        public double driveVelo = 0.0;
        public double drivePositionRads = 0.0;
        public double angleAbsoluteRads = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocity", driveVelo);
            table.put("DrivePositionRads", drivePositionRads);
            table.put("AngleAbsoluteRads", angleAbsoluteRads);
            table.put("AngleAbsoluteDegrees", Units.radiansToDegrees(angleAbsoluteRads));
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelo = table.get("DriveVelocity", driveVelo);
            drivePositionRads = table.get("DrivePositionRads", drivePositionRads);
            angleAbsoluteRads = table.get("AngleAbsoluteRads", angleAbsoluteRads);
        }
    }

    /**
     * @param desiredState The state that the module should assume, angle and
     *                     velocity.
     * @param isOpenLoop   Whether the module speed assumed should be reached via
     *                     open or closed loop control.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    /**
     * @return The velocity/angle of the module.
     */
    public SwerveModuleState getCurrentState();

    public SwerveModulePosition getCurrentPosition();

    /**
     * @return Returns the module's assigned number in the {@link Swerve#swerveMods}
     *         array.
     */
    public int getModuleNumber();

    default public void periodic() {
    }
}