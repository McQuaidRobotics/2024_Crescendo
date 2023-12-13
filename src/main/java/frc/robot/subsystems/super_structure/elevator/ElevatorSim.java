package frc.robot.subsystems.super_structure.elevator;

import frc.robot.Constants.kSuperStructure.kElevator;
import frc.robot.util.SimHelper.SimplePoseSim;;

public class ElevatorSim implements Elevator {

    private final Double maxVelo = (112.0 * kElevator.MOTOR_TO_MECHANISM_RATIO)
            * (kElevator.MECHANISM_DIAMETER_METERS * Math.PI);
    private final SimplePoseSim elevatorMeters = new SimplePoseSim(maxVelo);

    public ElevatorSim(Double startingMeters) {
        elevatorMeters.instantSetPose(startingMeters);
    }

    @Override
    public Boolean setElevatorMeters(Double meters) {
        elevatorMeters.setTargetPosition(meters);
        return Math.abs(elevatorMeters.getPose() - meters) < 0.1;
    }

    @Override
    public Double getElevatorMeters() {
        return elevatorMeters.getPose();
    }

    @Override
    public void manualDriveMechanism(Double percentOut) {
        elevatorMeters.setTargetVelocity(percentOut * maxVelo);
    }

    @Override
    public void stopMechanism() {
        elevatorMeters.setTargetPosition(elevatorMeters.getPose());
    }

    @Override
    public Boolean isLimitSwitchHit() {
        return elevatorMeters.getPose() < kElevator.HOME_METERS + 0.05;
    }

    @Override
    public Boolean homeMechanism(boolean force) {
        elevatorMeters.setTargetPosition(kElevator.HOME_METERS);
        return isLimitSwitchHit();
    }

    @Override
    public Double getRecentCurrent() {
        return 0.0;
    }
}