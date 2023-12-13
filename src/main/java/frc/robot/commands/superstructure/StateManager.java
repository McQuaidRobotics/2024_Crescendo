package frc.robot.commands.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GamepieceMode;
import frc.robot.commands.superstructure.Transitions.TransitionData;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.super_structure.States.IntakeBehavior;
import frc.robot.subsystems.super_structure.States.IntakeRequest;

/**
 * Acts as a stateful interface for the {@link SuperStructure}.
 * State transitions can be defined in {@link Transitions}.
 * The only public member is the {@link CmdTransitionState} command and should
 * be used as the main way of controlling the {@link SuperStructure}.
 */
public class StateManager {

    /**
     * Is essential for determining transitions,
     * can only be mutated by {@link CmdTransitionState}
     */
    private static States lastState;

    /**
     * Runs the given command when transitioning to the given state from the included states
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     * @param include The states to include in the transition
     *              WARNING: be careful of the order you call the methods in
     */
    private static void toStates(States state, Function<TransitionData, Command> cmd, States... include) {
        HashMap<States, Function<TransitionData, Command>> map = new HashMap<>();
        for (var iState : include) {
            map.put(iState, cmd);
        }
        transitions.put(state, map);
    }

    /**
     * Runs the given command when transitioning to the given state from any state
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     *              WARNING: be careful of the order you call the methods in
     */
    @SuppressWarnings("unused")
    private static void toAllStates(States state, Function<TransitionData, Command> cmd) {
        toStates(state, cmd, States.values());
    }

    /**
     * Sets the transition from all states to the given state to the given command
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     * @param include The states to include in the transition
     *              WARNING: be careful of the order you call the methods in
     */
    private static void fromStates(States state, Function<TransitionData, Command> cmd, States... include) {
        for (var iState : include) {
            if (iState == state)
                continue;
            if (transitions.containsKey(iState)) {
                transitions.get(iState).put(state, cmd);
            } else {
                HashMap<States, Function<TransitionData, Command>> newMap = new HashMap<>();
                newMap.put(state, cmd);
                transitions.put(iState, newMap);
            }
        }
    }

    /**
     * Sets the transition from the included states to the given state to the given command
     * 
     * @param state The state to transition to
     * @param cmd   The command to run when transitioning
     *              WARNING: be careful of the order you call the methods in
     */
    private static void fromAllStates(States state, Function<TransitionData, Command> cmd) {
        fromStates(state, cmd, States.values());
    }

    /**
     * A map of all possible transitions from one state to another,
     * default is simply setting the motors to the states' setpoints
     * WARNING: be careful of the order you edit the map in
     */
    private static Map<States, Map<States, Function<TransitionData, Command>>> transitions = new HashMap<>();

    static {
        // without this the superstructure will never reseed
        fromAllStates(States.HOME, Transitions::homeTransition);
        fromAllStates(States.STOW, Transitions::stowTransition);
        fromAllStates(States.PLACE_HIGH, Transitions::placeHighTransition);
        fromStates(States.PLACE_LOW_FRONT, Transitions::placeLowAntiChopTransition, States.STOW, States.HOME, States.PICKUP_GROUND);
    }

    /**
     * @param data The data to use for the transition
     * @return The command to run when transitioning from one state to another
     */
    private static Command getTransitionCmd(TransitionData data) {
        if (transitions.containsKey(data.from)) {
            if (transitions.get(data.from).containsKey(data.to)) {
                return transitions.get(data.from).get(data.to).apply(data);
            }
        }
        return Transitions.defaultTransition(data);
    }

    /**
     * A function to solve for the wanted voltage of the intake
     * @param req The intake request
     * @param useHeld Whether or not to use the held gamepiece variable or desired gamepiece variable
     * @return The wanted voltage of the intake
     */
    private static Double intakeVoltage(IntakeRequest req, Boolean useHeld) {
        if (useHeld) {
            var held = GamepieceMode.getHeldPiece();
            if (held == null) {
                return 0.0;
            }
            return req.getVoltage(held);
        } else {
            return req.getVoltage(
                GamepieceMode.getDesiredPiece()
            );
        }
    }

    /**
     * A function to solve for the wanted voltage of the intake
     * @param to The state to transition to
     * @return The wanted voltage of the intake
     */
    private static Double intakeVoltage(States to) {
        return intakeVoltage(to.intakeRequest, to.useHeldGamepiece);
    }

    /**
     * A Complex Command that handles calling state transitions and handling the intake logic
     */
    public static class CmdTransitionState extends CommandBase {
        private final SuperStructure superStructure;
        private final States to;
        private States from;
        private Command innerCmd;
        private Boolean canFinish = false;

        /**
         * Can only be set in initialize, will skip x many cycles,
         * this also delays inner cmd initialize
         */
        private Integer deadCycles = 0;

        // inner cmd tracking
        private Boolean innerInit = false;
        private Boolean innerFinish = false;

        private Boolean reachedSetpoint;

        public CmdTransitionState(final SuperStructure superStructure, final States to) {
            this.superStructure = superStructure;
            this.to = to;
            addRequirements(superStructure);
        }

        @Override
        public void initialize() {
            this.from = lastState;
            lastState = to;
            this.innerCmd = getTransitionCmd(new TransitionData(from, to, superStructure));
            this.innerInit = false;
            this.innerFinish = false;
            this.reachedSetpoint = false;
            this.deadCycles = 0;
            if (from == null) return;
            if (from.intakeBehavior == IntakeBehavior.RUN_ON_TRANSITION
                    && to.intakeBehavior != IntakeBehavior.RUN_ON_TRANSITION) {
                superStructure.runEndEffector(intakeVoltage(from), from.intakeRequest.getCurrentLimit());
                if (from.intakeRequest.expelling) {
                    GamepieceMode.setHeldPiece(null);
                }
                this.deadCycles = 15;
            }
        }

        @Override
        public void execute() {
            // skipping dead cycles
            if (deadCycles > 0) {
                deadCycles--;
                return;
            }

            // inner command handling
            if (!innerInit) {
                this.innerCmd.initialize();
                this.innerInit = true;
            }
            if (!innerFinish) {
                this.innerCmd.execute();
            }
            if (innerCmd.isFinished()) {
                this.innerCmd.end(false);
                this.innerFinish = true;
            }

            if (!this.reachedSetpoint && superStructure.reachedSetpoint(to.toleranceMult)) {
                this.reachedSetpoint = true;
            }

            // solving intake behavior
            Double endEffectorVolts = 0.0;
            Double endEffrctorAmps = 0.0;

            if (to.intakeBehavior == IntakeBehavior.RUN_ON_TRANSITION) {
                endEffrctorAmps = 15.0;
            }

            if (to.intakeBehavior == IntakeBehavior.RUN_WHOLE_TIME
                    || (to.intakeBehavior == IntakeBehavior.RUN_ON_START && !this.reachedSetpoint)) {
                // SmartDashboard.putString("intake run type", "START/WHOLE");
                endEffectorVolts = intakeVoltage(to);
                endEffrctorAmps = to.intakeRequest.maxCurrent;
            }
            if (reachedSetpoint) {
                if (to.intakeBehavior == IntakeBehavior.RUN_ON_START) {
                    // SmartDashboard.putString("intake run type", "stop cuz not start");
                    endEffectorVolts = 0.0;
                } else if (to.intakeBehavior == IntakeBehavior.RUN_ON_REACH) {
                    // SmartDashboard.putString("intake run type", "Run on reach");
                    endEffectorVolts = intakeVoltage(to);
                    endEffrctorAmps = to.intakeRequest.maxCurrent;
                }
            }

            if (endEffectorVolts == 0.0) {
                endEffectorVolts = intakeVoltage(IntakeRequest.HOLD, to.useHeldGamepiece);
            }

            superStructure.runEndEffector(endEffectorVolts, endEffrctorAmps);
        }

        @Override
        public void end(boolean interrupted) {
            if (!innerFinish) {
                this.innerCmd.end(interrupted);
            }
            superStructure.stopAll();
            from = null;
        }

        @Override
        public boolean isFinished() {
            if (canFinish) {
                return reachedSetpoint;
            }
            return false;
        }

        @Override
        public String getName() {
            if (from == null) {
                return "CmdTransitionState(? -> " + to + ")";
            } else {
                return "CmdTransitionState(" + from + " -> " + to + ")";
            }
        }

        public CmdTransitionState canFinish() {
            this.canFinish = true;
            return this;
        }
    }

    public static Command dispellGamepiece(SuperStructure superStructure) {
        return superStructure.startEnd(
                () -> {
                    if (lastState != null) {
                        superStructure.runEndEffector(intakeVoltage(lastState), lastState.intakeRequest.maxCurrent);
                    }
                },
                () -> {
                    superStructure.runEndEffector(0.0, 0.0);
                    GamepieceMode.setHeldPiece(null);
                    lastState = null;
                }
            ).withTimeout(0.25);
    }
    

    public static Command setLastState(States state) {
        return Commands.runOnce(() -> lastState = States.STANDBY);
    }
}
