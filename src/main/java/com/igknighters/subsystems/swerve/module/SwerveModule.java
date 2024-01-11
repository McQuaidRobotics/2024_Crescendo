package com.igknighters.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule extends Component {
    public static class SwerveModuleInputs implements LoggableInputs {
        public double driveVeloMPS = 0.0;
        public double drivePositionMeters = 0.0;
        public double driveVolts = 0.0;
        public double driveAmps = 0.0;
        public double angleVeloRadPS = 0.0;
        public double angleAbsoluteRads = 0.0;
        public double angleVolts = 0.0;
        public double angleAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocityMPS", driveVeloMPS);
            table.put("DrivePositionMeters", drivePositionMeters);
            table.put("DriveVolts", driveVolts);
            table.put("DriveAmps", driveAmps);
            table.put("AngleVelocityRPS", angleVeloRadPS);
            table.put("AngleAbsoluteRadians", angleAbsoluteRads);
            table.put("AngleVolts", angleVolts);
            table.put("AngleAmps", angleAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            driveVeloMPS = table.get("DriveVelocityMPS", driveVeloMPS);
            drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
            driveVolts = table.get("DriveVolts", driveVolts);
            driveAmps = table.get("DriveAmps", driveAmps);
            angleVeloRadPS = table.get("AngleVelocityRPS", angleVeloRadPS);
            angleAbsoluteRads = table.get("AngleAbsoluteRadians", angleAbsoluteRads);
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
}