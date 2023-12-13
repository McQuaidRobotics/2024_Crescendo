package frc.robot.subsystems.super_structure.wrist;

import frc.robot.subsystems.super_structure.Component;

public interface Wrist extends Component {
    /**
     * Abstract from motors, will set the wrist degrees.
     * Parallel to the elevator is 0 degrees
     * 
     * @return true if degrees has been reached
     */
    public Boolean setWristDegrees(Double degrees);

    /**
     * @return the current angle of the mechanism
     */
    public Double getWristDegrees();

    /**
     * Runs the intake at a given percent output
     * 
     * @param volts of the intake motor
     */
    public void runIntake(Double volts);

    /**
     * @return the voltage of the intake motor
     */
    public Double getIntakeVoltage();

    /**
     * @param amps true to enable current limits, false to disable
     */
    public void setIntakeCurrentLimits(Double amps);
}
