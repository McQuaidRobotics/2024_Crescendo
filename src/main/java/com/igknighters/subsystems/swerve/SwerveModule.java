package com.igknighters.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public static class SwerveModuleInputs implements LoggableInputs {
        public double driveVelo = 0.0;
        public double drivePosition = 0.0;
        public double driveVolts = 0.0;
        public double driveAmps = 0.0;
        public double angleVelo = 0.0;
        public double angleAbsolute = 0.0;
        public double angleVolts = 0.0;
        public double angleAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocity", driveVelo);
            table.put("DrivePosition", drivePosition);
            table.put("DriveVolts", driveVolts);
            table.put("DriveAmps", driveAmps);
            table.put("AngleVelocity", angleVelo);
            table.put("AngleAbsolute", angleAbsolute);
            table.put("AngleVolts", angleVolts);
            table.put("AngleAmps", angleAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            driveVelo = table.get("DriveVelocity", driveVelo);
            drivePosition = table.get("DrivePosition", drivePosition);
            driveVolts = table.get("DriveVolts", driveVolts);
            driveAmps = table.get("DriveAmps", driveAmps);
            angleVelo = table.get("AngleVelocity", angleVelo);
            angleAbsolute = table.get("AngleAbsolute", angleAbsolute);
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

    default public void periodic() {
    }
}