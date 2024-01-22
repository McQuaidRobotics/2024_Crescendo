package com.igknighters.subsystems.stem;

import com.igknighters.subsystems.stem.pivot.*;
import com.igknighters.subsystems.stem.telescope.*;
import com.igknighters.subsystems.stem.wrist.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stem extends SubsystemBase {

    private final Pivot pivot;
    private final Telescope telescope;
    private final Wrist wrist;

    public Stem() {
        if (RobotBase.isSimulation()) {
            pivot = new PivotSim();
            telescope = new TelescopeDisabled();
            wrist = new WristSim();
        } else {
            pivot = new PivotReal();
            telescope = new TelescopeDisabled();
            wrist = new WristReal();
        }
    }

    /**
     * Meant as the main api for controlling the stem,
     * this method takes in a {@link StemPositions.StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * @param position The position to set the stem to
     * @return True if all mechanisms have reached their target position
     */
    public boolean setStemPosition(StemPositions.StemPosition position) {
        boolean pivotSuccess = pivot.target(position.pivotPosRads);
        boolean wristSuccess = wrist.target(position.wristPosRads);
        boolean telescopeSuccess = telescope.target(position.telescopePosMeters);
        return pivotSuccess && wristSuccess && telescopeSuccess;
    }

    /**
     * @return The current position of the stem
     */
    public StemPositions.StemPosition getStemPosition() {
        return StemPositions.StemPosition.fromRadians(
            pivot.getPivotRadians(),
            wrist.getWristRadians(),
            telescope.getTelescopeMeters()
        );
    }

    /**
     * Immedaitely cuts off the voltage to all mechanisms causing them to stop,
     * this is not recommended for general use.
     */
    public void stopMechanisms() {
        pivot.stopMechanism();
        telescope.stopMechanism();
        wrist.stopMechanism();
    }

    /**
     * Control each component with voltage control.
     * This is NOT recommended for general use and should only be used to test
     * the mechanisms.
     */
    public void setStemVolts(double pivotVolts, double wristVolts, double telescopeVolts) {
        pivot.setVoltageOut(pivotVolts);
        wrist.setVoltageOut(wristVolts);
        telescope.setVoltageOut(telescopeVolts);
    }

    @Override
    public void periodic() {
        pivot.periodic();
        telescope.periodic();
        wrist.periodic();
    }
}
