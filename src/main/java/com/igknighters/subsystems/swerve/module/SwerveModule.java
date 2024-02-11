package com.igknighters.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.igknighters.subsystems.Component;
import com.igknighters.subsystems.swerve.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule extends Component {
    public static class SwerveModuleInputs implements LoggableInputs {
        /**Measured in Meters/s */
        public double driveVeloMPS = 0.0;
        public double targetDriveVeloMPS = 0.0;
        /**Measured in Meters */
        public double drivePositionMeters = 0.0;
        public double driveVolts = 0.0;
        public double driveAmps = 0.0;
        /**Measured in Radian/s */
        public double angleVeloRadPS = 0.0;
        /**Measured in Radians */
        public double angleAbsoluteRads = 0.0;
        public double targetAngleAbsoluteRads = 0.0;
        public double angleVolts = 0.0;
        public double angleAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DriveVelocityMPS", driveVeloMPS);
            table.put("TargetDriveVelocityMPS", targetDriveVeloMPS);
            table.put("DrivePositionMeters", drivePositionMeters);
            table.put("DriveVolts", driveVolts);
            table.put("DriveAmps", driveAmps);
            table.put("AngleVelocityRadPS", angleVeloRadPS);
            table.put("AngleAbsoluteRadians", angleAbsoluteRads);
            table.put("TargetAngleAbsoluteRadians", targetAngleAbsoluteRads);
            table.put("AngleVolts", angleVolts);
            table.put("AngleAmps", angleAmps);
            table.put("AngleAbsoluteDegrees", angleAbsoluteRads * (180/Math.PI));
        }

        @Override
        public void fromLog(LogTable table) {
            driveVeloMPS = table.get("DriveVelocityMPS", driveVeloMPS);
            targetDriveVeloMPS = table.get("TargetDriveVelocityMPS", targetDriveVeloMPS);
            drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
            driveVolts = table.get("DriveVolts", driveVolts);
            driveAmps = table.get("DriveAmps", driveAmps);
            angleVeloRadPS = table.get("AngleVelocityRadPS", angleVeloRadPS);
            angleAbsoluteRads = table.get("AngleAbsoluteRadians", angleAbsoluteRads);
            targetAngleAbsoluteRads = table.get("TargetAngleAbsoluteRadians", targetAngleAbsoluteRads);
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