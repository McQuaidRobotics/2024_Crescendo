package com.igknighters.subsystems.stem;

import com.igknighters.subsystems.stem.pivot.*;
import com.igknighters.subsystems.stem.telescope.*;
import com.igknighters.subsystems.stem.wrist.*;
import com.igknighters.util.Tracer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stem extends SubsystemBase {

    private final Pivot pivot;
    private final Telescope telescope;
    private final Wrist wrist;

    private final StemVisualizer visualizer;

    public Stem() {
        if (RobotBase.isSimulation()) {
            pivot = new PivotSim();
            telescope = new TelescopeDisabled();
            wrist = new WristSim();
        } else {
            pivot = new PivotSim();
            telescope = new TelescopeDisabled();
            wrist = new WristReal();
        }

        visualizer = new StemVisualizer();
    }

    /**
     * Meant as the main api for controlling the stem,
     * this method takes in a {@link StemPosition.StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * 
     * @param position The position to set the stem to
     * @param toleranceMult A value to multiply the accepted positional tolerance by
     * @return True if all mechanisms have reached their target position
     */
    public boolean setStemPosition(StemPosition position, double toleranceMult) {
        visualizer.updateSetpoint(position);
        boolean pivotSuccess = pivot.target(position.pivotRads, toleranceMult);
        boolean wristSuccess = wrist.target(position.wristRads, toleranceMult);
        boolean telescopeSuccess = telescope.target(position.telescopeMeters, toleranceMult);
        return pivotSuccess && wristSuccess && telescopeSuccess;
    }

    /**
     * Meant as the main api for controlling the stem,
     * this method takes in a {@link StemPosition.StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * 
     * @param position The position to set the stem to
     * @return True if all mechanisms have reached their target position
     */
    public boolean setStemPosition(StemPosition position) {
        return setStemPosition(position, 1.0);
    }

    /**
     * @return The current position of the stem
     */
    public StemPosition getStemPosition() {
        return StemPosition.fromRadians(
                pivot.getPivotRadians(),
                wrist.getWristRadians(),
                telescope.getTelescopeMeters());
    }

    /**
     * Immedaitely cuts off the voltage to all mechanisms causing them to stop,
     * this is not recommended for general use.
     */
    public void stopMechanisms() {
        visualizer.updateSetpoint(getStemPosition());
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
        Tracer.startTrace("StemPeriodic");

        Tracer.traceFunc("PivotPeriodic", pivot::periodic);
        Tracer.traceFunc("TelescopePeriodic", telescope::periodic);
        Tracer.traceFunc("WristPeriodic", wrist::periodic);

        visualizer.updateCurrent(getStemPosition());

        Tracer.endTrace();
    }
}
