package frc.robot.subsystems.super_structure.elevator;

import frc.robot.subsystems.super_structure.Component;
import frc.robot.Constants.kSuperStructure.kElevator;

public interface Elevator extends Component {
    /**
     * Abstract from motors, will set the elevator meters.
     * 
     * @param meters from pivot
     *               (min: {@link kElevator#ELEVATOR_MIN_METERS},
     *               max: {@link kElevator#ELEVATOR_MAX_METERS})
     * @return true if meters has been reached
     */
    public Boolean setElevatorMeters(Double meters);

    public Double getElevatorMeters();

    /**
     * @return if the reverse limit switch is activated
     */
    public Boolean isLimitSwitchHit();
}