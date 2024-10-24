package com.igknighters.controllers;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.igknighters.Robot;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.SubsystemResources.Subsystems;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerBase {

    private final CommandXboxController controller;
    private boolean madeController;

    protected class TriggerBindingTuple {
        public final Trigger trigger;
        public Binding binding;

        public TriggerBindingTuple(Trigger trigger, Binding binding) {
            this.trigger = trigger;
            this.binding = binding;
        }
    }

    protected static class Binding {
        private static final Binding EMPTY = new Binding((trig, allss) -> {
        });

        public final List<Subsystems> subsystemArray;
        public final BiConsumer<Trigger, AllSubsystems> action;
        private boolean isBound = false;

        public Binding(Subsystems[] subsystemArray, BiConsumer<Trigger, AllSubsystems> action) {
            this.subsystemArray = Arrays.asList(subsystemArray);
            this.action = action;
        }

        public Binding(Subsystems subsystem, BiConsumer<Trigger, AllSubsystems> action) {
            this.subsystemArray = List.of(subsystem);
            this.action = action;
        }

        public Binding(BiConsumer<Trigger, AllSubsystems> action, Subsystems... subsystemArray) {
            this.subsystemArray = Arrays.asList(subsystemArray);
            this.action = action;
        }

        public boolean hasDeps(HashSet<Subsystems> enabledSubsystems) {
            for (Subsystems subsystem : subsystemArray) {
                if (!enabledSubsystems.contains(subsystem)) {
                    return false;
                }
            }
            return true;
        }

        public void assign(Trigger trigger, AllSubsystems subsystems) {
            action.accept(trigger, subsystems);
            isBound = true;
        }

        public boolean isBound() {
            return isBound;
        }

        public static Binding empty() {
            return EMPTY;
        }
    }

    /** Button: 1 */
    protected final TriggerBindingTuple A;
    /** Button: 2 */
    protected final TriggerBindingTuple B;
    /** Button: 3 */
    protected final TriggerBindingTuple X;
    /** Button: 4 */
    protected final TriggerBindingTuple Y;
    /** Left Center; Button: 7 */
    protected final TriggerBindingTuple Back;
    /** Right Center; Button: 8 */
    protected final TriggerBindingTuple Start;
    /** Left Bumper; Button: 5 */
    protected final TriggerBindingTuple LB;
    /** Right Bumper; Button: 6 */
    protected final TriggerBindingTuple RB;
    /** Left Stick; Button: 9 */
    protected final TriggerBindingTuple LS;
    /** Right Stick; Button: 10 */
    protected final TriggerBindingTuple RS;
    /** Left Trigger; Axis: 2  */
    protected final TriggerBindingTuple LT;
    /** Right Trigger; Axis: 3 */
    protected final TriggerBindingTuple RT;
    /** DPad Up; Degrees: 0 */
    protected final TriggerBindingTuple DPU;
    /** DPad Right; Degrees: 90 */
    protected final TriggerBindingTuple DPR;
    /** DPad Down; Degrees: 180 */
    protected final TriggerBindingTuple DPD;
    /** DPad Left; Degrees: 270 */
    protected final TriggerBindingTuple DPL;


    /**
     * for button idx (nice for sim)
     * {@link edu.wpi.first.wpilibj.XboxController.Button}
     */
    protected ControllerBase(int port, boolean makeController) {
        DriverStation.silenceJoystickConnectionWarning(Robot.isDebug());
        this.madeController = makeController;
        if (madeController) {
            controller = new CommandXboxController(port);
            BootupLogger.bootupLog("Controller " + port + " initialized");
        } else {
            controller = null;
            BootupLogger.bootupLog("Controller " + port + " not initialized");
            final Trigger t = new Trigger(() -> false);
            A = new TriggerBindingTuple(t, Binding.empty());
            B = new TriggerBindingTuple(t, Binding.empty());
            X = new TriggerBindingTuple(t, Binding.empty());
            Y = new TriggerBindingTuple(t, Binding.empty());
            LB = new TriggerBindingTuple(t, Binding.empty());
            RB = new TriggerBindingTuple(t, Binding.empty());
            Back = new TriggerBindingTuple(t, Binding.empty());
            Start = new TriggerBindingTuple(t, Binding.empty());
            LS = new TriggerBindingTuple(t, Binding.empty());
            RS = new TriggerBindingTuple(t, Binding.empty());
            LT = new TriggerBindingTuple(t, Binding.empty());
            RT = new TriggerBindingTuple(t, Binding.empty());
            DPR = new TriggerBindingTuple(t, Binding.empty());
            DPD = new TriggerBindingTuple(t, Binding.empty());
            DPL = new TriggerBindingTuple(t, Binding.empty());
            DPU = new TriggerBindingTuple(t, Binding.empty());
            return;
        }
        A = new TriggerBindingTuple(controller.a(), Binding.empty());
        B = new TriggerBindingTuple(controller.b(), Binding.empty());
        X = new TriggerBindingTuple(controller.x(), Binding.empty());
        Y = new TriggerBindingTuple(controller.y(), Binding.empty());
        LB = new TriggerBindingTuple(controller.leftBumper(), Binding.empty());
        RB = new TriggerBindingTuple(controller.rightBumper(), Binding.empty());
        Back = new TriggerBindingTuple(controller.back(), Binding.empty());
        Start = new TriggerBindingTuple(controller.start(), Binding.empty());
        LS = new TriggerBindingTuple(controller.leftStick(), Binding.empty());
        RS = new TriggerBindingTuple(controller.rightStick(), Binding.empty());
        LT = new TriggerBindingTuple(controller.leftTrigger(0.25), Binding.empty());
        RT = new TriggerBindingTuple(controller.rightTrigger(0.25), Binding.empty());
        DPR = new TriggerBindingTuple(controller.povRight(), Binding.empty());
        DPD = new TriggerBindingTuple(controller.povDown(), Binding.empty());
        DPL = new TriggerBindingTuple(controller.povLeft(), Binding.empty());
        DPU = new TriggerBindingTuple(controller.povUp(), Binding.empty());
    }

    public void assignButtons(AllSubsystems subsystems) {
        if (!madeController) {
            return;
        }
        HashSet<Subsystems> subsystemSet = new HashSet<Subsystems>(
                Arrays.asList(subsystems.getEnabledSubsystemEnums()));
        TriggerBindingTuple[] tuples = new TriggerBindingTuple[] {
                A, B, X, Y, LB, RB, Back, Start, LS, RS, LT, RT, DPR, DPD, DPL, DPU };
        for (int i = 0; i < tuples.length; i++) {
            TriggerBindingTuple tuple = tuples[i];
            if (tuple.binding.hasDeps(subsystemSet)) {
                tuple.binding.assign(tuple.trigger, subsystems);
            }
        }
    }

    private DoubleSupplier deadbandSupplier(DoubleSupplier supplier, double deadband) {
        return () -> {
            double val = supplier.getAsDouble();
            if (Math.abs(val) > deadband) {
                if (val > 0.0) {
                    val = (val - deadband) / (1.0 - deadband);
                } else {
                    val = (val + deadband) / (1.0 - deadband);
                }
            } else {
                val = 0.0;
            }
            return val;
        };
    }

    /**
     * Right on the stick is positive (axis 4)
     * 
     * @return A supplier for the value of the right stick x axis
     */
    public DoubleSupplier rightStickX() {
        return () -> -controller.getRightX();
    }

    /**
     * Right on the stick is positive (axis 4)
     * 
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick x axis
     */
    public DoubleSupplier rightStickX(double deadband) {
        return deadbandSupplier(rightStickX(), deadband);
    }

    /**
     * Up on the stick is positive (axis 5)
     * 
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier rightStickY() {
        return controller::getRightY;
    }

    /**
     * Up on the stick is positive (axis 5)
     * 
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the right stick y axis
     */
    public DoubleSupplier rightStickY(double deadband) {
        return deadbandSupplier(rightStickY(), deadband);
    }

    /**
     * Right on the stick is positive (axis 0)
     * 
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier leftStickX() {
        return () -> -controller.getLeftX();
    }

    /**
     * Right on the stick is positive (axis 0)
     * 
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the left stick x axis
     */
    public DoubleSupplier leftStickX(double deadband) {
        return deadbandSupplier(leftStickX(), deadband);
    }

    /**
     * Up on the stick is positive (axis 1)
     * 
     * @return A supplier for the value of the left stick y axis
     */
    public DoubleSupplier leftStickY() {
        return controller::getLeftY;
    }

    /**
     * Up on the stick is positive (axis 1)
     * 
     * @param deadband the deadband to apply to the stick
     * @return A supplier for the value of the left stick y axis
     */
    public DoubleSupplier leftStickY(double deadband) {
        return deadbandSupplier(leftStickY(), deadband);
    }

    /**
     * will print warning if this trigger is also bound to a command
     * 
     * @param suppressWarning if true will not print warning even if bound to a
     *                        command
     */
    public DoubleSupplier rightTrigger(boolean suppressWarning) {
        if (RT.binding.isBound() && !suppressWarning) {
            return () -> {
                DriverStation.reportError("WARNING: Right Trigger is bound to a command", false);
                return controller.getRightTriggerAxis();
            };
        } else {
            return controller::getRightTriggerAxis;
        }
    }

    /**
     * will print warning if this trigger is also bound to a command
     * 
     * @param suppressWarning if true will not print warning even if bound to a
     *                        command
     */
    public DoubleSupplier leftTrigger(boolean suppressWarning) {
        if (LT.binding.isBound() && !suppressWarning) {
            return () -> {
                DriverStation.reportError("WARNING: Left Trigger is bound to a command", false);
                return controller.getLeftTriggerAxis();
            };
        } else {
            return controller::getLeftTriggerAxis;
        }
    }

    /**
     * Will rumble both sides of the controller with a magnitude
     * 
     * @param magnitude The magnitude to rumble at
     */
    public void rumble(double magnitude) {
        controller.getHID().setRumble(RumbleType.kBothRumble, magnitude);
    }
}
