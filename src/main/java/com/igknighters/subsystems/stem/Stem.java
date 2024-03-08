
package com.igknighters.subsystems.stem;

import com.igknighters.GlobalState;
import com.igknighters.LED;
import com.igknighters.LED.LedAnimations;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.subsystems.stem.StemValidator.ValidationResponse;
import com.igknighters.subsystems.stem.pivot.*;
import com.igknighters.subsystems.stem.telescope.*;
import com.igknighters.subsystems.stem.wrist.*;
import com.igknighters.util.CANBusLogging;
import com.igknighters.util.Tracer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;

public class Stem extends SubsystemBase implements Logged {

    private final Pivot pivot;
    private final Telescope telescope;
    private final Wrist wrist;

    private final StemVisualizer visualizer;

    private final DigitalInput coastSwitch;

    public Stem() {
        if (RobotBase.isSimulation()) {
            pivot = new PivotDisabled();
            telescope = new TelescopeDisabled();
            wrist = new WristDisabled();
        } else {
            pivot = new PivotReal();
            telescope = new TelescopeReal();
            wrist = new WristReal();
        }

        visualizer = new StemVisualizer();

        if (!GlobalState.isUnitTest()) {
            coastSwitch = new DigitalInput(kStem.COAST_SWITCH_CHANNEL);
            new Trigger(coastSwitch::get)
                    .and(DriverStation::isDisabled)
                    .or(DriverStation::isTestEnabled)
                    .onTrue(Commands.runOnce(() -> {
                        pivot.setCoast(true);
                        telescope.setCoast(true);
                        wrist.setCoast(true);
                    }).ignoringDisable(true))
                    .onFalse(Commands.runOnce(() -> {
                        pivot.setCoast(false);
                        telescope.setCoast(false);
                        wrist.setCoast(false);
                    }).ignoringDisable(true));
        } else {
            coastSwitch = null;
        }

        CANBusLogging.logBus(kStem.CANBUS);
    }

    /**
     * Meant as the main api for controlling the stem,
     * this method takes in a {@link StemPosition} and sets the
     * stem to that position. This method will return false if any of the
     * mechanisms have not yet reached their target position.
     * 
     * @param position      The position to set the stem to
     * @param toleranceMult A value to multiply the accepted positional tolerance by
     * @return True if all mechanisms have reached their target position
     */
    public boolean setStemPosition(StemPosition position, double toleranceMult) {
        visualizer.updateSetpoint(position);

        ValidationResponse validity = StemValidator.validatePosition(position);

        if (!validity.isValid()) {
            DriverStation.reportError(
                    "Invalid TARGET stem position(" + validity.name() + "): " + position.toString(),
                    true);
            return true;
        }

        if (!telescope.hasHomed()) {
            if (!position.isStow()) {
                DriverStation.reportWarning("Stem Telescope has not been homed, run stow to home", false);
                LED.sendAnimation(LedAnimations.WARNING).withDuration(1.0);
                return false;
            }
            boolean wristAndPivot = pivot.target(position.pivotRads, 1.0)
                    && wrist.target(position.wristRads, 1.0);

            if (wristAndPivot) {
                telescope.setVoltageOut(-4.0);
            }

            return telescope.hasHomed();
        }

        StemPosition validated = StemValidator.stepTowardsTargetPosition(getStemPosition(), position, 1.0);
        pivot.setPivotRadians(validated.pivotRads);
        telescope.setTelescopeMeters(validated.telescopeMeters);
        wrist.setWristRadians(validated.wristRads);

        boolean pivotSuccess = pivot.isAt(position.pivotRads, toleranceMult);
        boolean telescopeSuccess = telescope.isAt(position.telescopeMeters, toleranceMult);
        boolean wristSuccess = wrist.isAt(position.wristRads, toleranceMult);

        log("/PivotReached", pivotSuccess);
        log("/TelescopeReached", telescopeSuccess);
        log("/WristReached", wristSuccess);

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

        log("CurrentPosition", getStemPosition());
        log("StemValidator/CurrentStateValidation",
                StemValidator.validatePosition(getStemPosition()).toString());

        visualizer.updateCurrent(getStemPosition());

        Tracer.endTrace();
    }
}
