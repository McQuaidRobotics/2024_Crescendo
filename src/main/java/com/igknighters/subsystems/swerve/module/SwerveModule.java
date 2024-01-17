package com.igknighters.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public static class SwerveModuleInputs implements LoggableInputs {
        /**Measured in Meters/s */
        public double driveVelo = 0.0;
        public double targetDriveVelo = 0.0;
        /**Measured in Meters */
        public double drivePosition = 0.0;
        public double driveVolts = 0.0;
        public double driveAmps = 0.0;
        /**Measured in Radian/s */
        public double angleVelo = 0.0;
        /**Measured in Radians */
        public double angleAbsolute = 0.0;
        public double targetAngleAbsolute = 0.0;
        public double angleVolts = 0.0;
        public double angleAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocityMPS", driveVelo);
            table.put("TargetDriveVelocityMPS", targetDriveVelo);
            table.put("DrivePositionMeters", drivePosition);
            table.put("DriveVolts", driveVolts);
            table.put("DriveAmps", driveAmps);
            table.put("AngleVelocityRPS", angleVelo);
            table.put("AngleAbsoluteRadians", angleAbsolute);
            table.put("TargetAngleAbsoluteRadians", targetAngleAbsolute);
            table.put("AngleVolts", angleVolts);
            table.put("AngleAmps", angleAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelo = table.get("DriveVelocityMPS", driveVelo);
            targetAngleAbsolute = table.get("TargetDriveVelocityMPS", targetDriveVelo);
            drivePosition = table.get("DrivePositionMeters", drivePosition);
            driveVolts = table.get("DriveVolts", driveVolts);
            driveAmps = table.get("DriveAmps", driveAmps);
            angleVelo = table.get("AngleVelocityRPS", angleVelo);
            angleAbsolute = table.get("AngleAbsoluteRadians", angleAbsolute);
            targetAngleAbsolute = table.get("TargetAngleAbsoluteRadians", targetAngleAbsolute);
            angleVolts = table.get("AngleVolts", angleVolts);
            angleAmps = table.get("AngleAmps", angleAmps);
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

    public void periodic();
}