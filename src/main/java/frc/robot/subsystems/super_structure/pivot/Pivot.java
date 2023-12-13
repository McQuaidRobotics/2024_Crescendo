package frc.robot.subsystems.super_structure.pivot;

import frc.robot.subsystems.super_structure.Component;

public interface Pivot extends Component {

    /**
     * Abstract from motors, will set the wrist degrees.
     * Parallel to the elevator is 0 degrees
     * 
     * @return true if degrees has been reached
     */
    public Boolean setPivotDegrees(Double degrees);

    /**
     * @return the current angle of the mechanism
     */
    public Double getPivotDegrees();
}
