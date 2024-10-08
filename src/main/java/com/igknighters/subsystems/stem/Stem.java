
package com.igknighters.subsystems.stem;

import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.subsystems.SubsystemResources.LockFullSubsystem;
import com.igknighters.subsystems.stem.StemValidator.ValidationResponse;
import com.igknighters.subsystems.stem.pivot.*;
import com.igknighters.subsystems.stem.telescope.*;
import com.igknighters.subsystems.stem.wrist.*;
import com.igknighters.util.logging.Tracer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Stem implements LockFullSubsystem {

    private final Pivot pivot;
    private final Telescope telescope;
    private final Wrist wrist;

    private final StemVisualizer visualizer;

    private final DigitalInput coastSwitch;

    public Stem() {
        if (Robot.isSimulation()) {
            pivot = new PivotDisabled();
            telescope = new TelescopeDisabled();
            wrist = new WristDisabled();
        } else {
            pivot = new PivotReal();
            if (Robot.isSunlight()) {
                telescope = new TelescopeRealSunshine();
            } else {
                telescope = new TelescopeReal();
            }
            wrist = new WristRealFused();
        }

        visualizer = new StemVisualizer();

        if (!Robot.isUnitTest()) {
            coastSwitch = new DigitalInput(kStem.COAST_SWITCH_CHANNEL);
            new Trigger(coastSwitch::get)
                    .and(DriverStation::isDisabled)
                    .or(DriverStation::isTestEnabled)
                    .onTrue(Commands.runOnce(() -> {
                        pivot.setCoast(true);
                        telescope.setCoast(true);
                        wrist.setCoast(true);
                    }).ignoringDisable(true)
                    .withName("CoastOn"))
                    .onFalse(Commands.runOnce(() -> {
                        pivot.setCoast(false);
                        telescope.setCoast(false);
                        wrist.setCoast(false);
                    }).ignoringDisable(true)
                    .withName("CoastOff"));
        } else {
            // Multiple instantiations on the same channel throws an except,
            // this code can run any number of times in a unit test
            coastSwitch = null;
        }
    }

    /**
     * Meant as the main entry point for controlling the stem,
     * this method takes in a {@link StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * 
     * @param position      The position to set the stem to
     * @param toleranceMult A value to multiply the accepted positional tolerance by
     * @return True if all mechanisms have reached their target position
     */
    public boolean gotoStemPosition(StemPosition position, double toleranceMult) {
        visualizer.updateSetpoint(position);

        ValidationResponse validity = StemValidator.validatePosition(position);

        // If the position is invalid, report an error and return true
        if (!validity.isValid()) {
            DriverStation.reportError(
                    "Invalid TARGET stem position(" + validity.name() + "): " + position.toString(),
                    true);
            return true;
        }

        // If the telescope has not been homed, we need to run the stow command
        if (!telescope.hasHomed()) {
            if (!position.isStow()) {
                DriverStation.reportWarning("Stem Telescope has not been homed, run stow to home", false);
                return false;
            }

            // First move the pivot and wrist to the stow position
            boolean wristAndPivot = pivot.target(position.pivotRads, 1.0)
                    && wrist.target(position.wristRads, 1.0);


            // Then drive the telescope down until we hit the limit switch
            // we have to use open loop as we don't know the absolute position of the telescope
            if (wristAndPivot) {
                telescope.setVoltageOut(-4.0);
            }

            // If the telescope has homed that means we are successfully in the stow position
            // and can return that we are at the target position
            return telescope.hasHomed();
        }

        // Generate and move to the next safe position
        StemPosition step = StemValidator.stepTowardsTargetPosition(getStemPosition(), position);
        pivot.gotoPosition(step.pivotRads);
        telescope.gotoPosition(step.telescopeMeters);
        wrist.gotoPosition(step.wristRads);

        // Query if the mechanisms have reached their target position
        boolean pivotSuccess = pivot.isAt(position.pivotRads, toleranceMult);
        boolean telescopeSuccess = telescope.isAt(position.telescopeMeters, toleranceMult);
        boolean wristSuccess = wrist.isAt(position.wristRads, toleranceMult);

        log("PivotReached", pivotSuccess);
        log("TelescopeReached", telescopeSuccess);
        log("WristReached", wristSuccess);

        // Return if all mechanisms have reached their target position
        return pivotSuccess && telescopeSuccess && wristSuccess;
    }

    /**
     * Meant as the main api for controlling the stem,
     * this method takes in a {@link StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * 
     * @param position The position to set the stem to
     * @return True if all mechanisms have reached their target position
     */
    public boolean gotoStemPosition(StemPosition position) {
        return gotoStemPosition(position, 1.0);
    }

    /**
     * Will check if all mechanisms have reached their target position
     * within a specific tolerance.
     * @param targetPosition The desired position
     * @param toleranceMult A value to multiply the accepted positional tolerance by
     * @@return True if all mechanisms have reached their target position
     */
    public boolean isAt(StemPosition targetPosition, double toleranceMult) {
        return pivot.isAt(targetPosition.pivotRads, toleranceMult)
                && telescope.isAt(targetPosition.telescopeMeters, toleranceMult)
                && wrist.isAt(targetPosition.wristRads, toleranceMult);
    }

    /**
     * Will check if all mechanisms have reached their target position
     * within a specific tolerance.
     * @param targetPosition The desired position
     * @@return True if all mechanisms have reached their target position
     */
    public boolean isAt(StemPosition targetPosition) {
        return isAt(targetPosition, 1.0);
    }

    /**
     * @return The current position of the stem
     */
    public StemPosition getStemPosition() {
        return StemPosition.fromRadians(
                pivot.getPosition(),
                wrist.getPosition(),
                telescope.getPosition());
    }

    /**
     * Immedaitely cuts off the voltage to all mechanisms causing them to stop,
     * this is not recommended for general use.
     */
    public void stopMechanisms() {
        visualizer.updateSetpoint(getStemPosition());
        pivot.setVoltageOut(0.0);
        telescope.setVoltageOut(0.0);
        wrist.setVoltageOut(0.0);
    }

    /**
     * Control each component with voltage control.
     * This is NOT recommended for general use and should only be used to test
     * the mechanisms.
     * 
     * @param pivotVolts     The voltage to run the pivot at
     * @param wristVolts     The voltage to run the wrist at
     * @param telescopeVolts The voltage to run the telescope at
     */
    public void setStemVolts(double pivotVolts, double wristVolts, double telescopeVolts) {
        pivot.setVoltageOut(pivotVolts);
        wrist.setVoltageOut(wristVolts);
        telescope.setVoltageOut(telescopeVolts);
    }

    /**
     * @return An array holding velocity values for [pivot, telescope, wrist]
     */
    public double[] getStemVelocities() {
        return new double[] {
            pivot.getVelocity(),
            telescope.getVelocity(),
            wrist.getVelocity()
        };
    }

    /**
     * Homes any of the mechanisms that need homing
     */
    public void home() {
        pivot.home();
    }

    /**
     * Returns if the stem is OK,
     * the stem can not be OK for a variety of reasons
     * such as the telescope not being homed,
     * hardware being dropped, or the stem being in an invalid position.
     */
    public boolean isOk() {
        return StemValidator.validatePosition(getStemPosition()).isValid()
                && telescope.hasHomed();
    }

    @Override
    public void periodic() {
        Tracer.startTrace("StemPeriodic");

        // This prevents the robot from moving again when re-enabling
        if (DriverStation.isDisabled()) {
            stopMechanisms();
        }

        // run and time the components periodic loops
        Tracer.traceFunc("PivotPeriodic", pivot::periodic);
        Tracer.traceFunc("TelescopePeriodic", telescope::periodic);
        Tracer.traceFunc("WristPeriodic", wrist::periodic);

        // run logging code after loops to have the most up to date information
        log("CurrentPosition", getStemPosition());
        log("StemValidator/CurrentStateValidation",
                StemValidator.validatePosition(getStemPosition()));

        Tracer.traceFunc(
            "Visualizer",
            () -> visualizer.updateCurrent(getStemPosition())
        );

        Tracer.endTrace();
    }
}
