package frc.robot.commands.superstructure;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GamepieceMode;
import frc.robot.commands.Helpers;
import frc.robot.commands.superstructure.StateManager.CmdTransitionState;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;

public class SuperstructureCommands {

    /** Inputs are halved */
    public static Command manualControl(
            SuperStructure superStructure,
            DoubleSupplier elevator,
            DoubleSupplier pivot,
            DoubleSupplier wrist,
            DoubleSupplier intake) {
        var dbElevator = Helpers.deadbandSupplier(elevator, 0.1);
        var dbPivot = Helpers.deadbandSupplier(pivot, 0.1);
        var dbWrist = Helpers.deadbandSupplier(wrist, 0.1);

        return superStructure.run(() -> {
            superStructure.manualControl(
                    dbWrist.getAsDouble() / 2.0,
                    dbPivot.getAsDouble() / 2.0,
                    dbElevator.getAsDouble() / 2.0,
                    intake.getAsDouble());
        });
    }

    /**
     * Acts as a pseudo-proxy command for
     * new {@link StateManager.CmdTransitionState}(ss, {@link States#PLACE_HIGH}),
     * new {@link StateManager.CmdTransitionState}(ss, {@link States#PLACE_MID}), or
     * new {@link StateManager.CmdTransitionState}(ss,
     * {@link States#PLACE_LOW_FRONT})
     * that will dynamically determine which to use based on
     * {@link OperatorPrefs.ScoreLevel#getCurrentLevel()}.
     * The decided proxy command can only be changed by calling
     * {@link #initialize()} again.
     */
    public static class TransitionToPlace extends CommandBase {
        private final SuperStructure superStructure;
        private CmdTransitionState placeCmd;
        private Boolean canFinish = false;

        public TransitionToPlace(final SuperStructure superStructure) {
            this.superStructure = superStructure;
            addRequirements(superStructure);
        }

        @Override
        public void initialize() {
            switch (OperatorPrefs.ScoreLevel.getCurrentLevel()) {
                case HIGH:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PLACE_HIGH);
                    break;
                case MID:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PLACE_MID);
                    break;
                case LOW_FRONT:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PLACE_LOW_FRONT);
                    break;
                case LOW_BACK:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PLACE_LOW_BACK);
                    break;
            }
            if (this.canFinish) {
                placeCmd = placeCmd.canFinish();
            }
            placeCmd.initialize();
        }

        @Override
        public void execute() {
            placeCmd.execute();
        }

        @Override
        public void end(boolean interrupted) {
            placeCmd.end(interrupted);
        }

        @Override
        public boolean isFinished() {
            return placeCmd.isFinished();
        }

        @Override
        public String getName() {
            if (placeCmd == null)
                return "TransitionToPlace(null)";
            return "TransitionToPlace(" + placeCmd.getName() + ")";
        }

        public TransitionToPlace canFinish() {
            this.canFinish = true;
            return this;
        }
    }

    /**
     * Acts as a pseudo-proxy command for
     * new {@link StateManager.CmdTransitionState}(ss, {@link States#PICKUP_GROUND})
     * or
     * new {@link StateManager.CmdTransitionState}(ss,
     * {@link States#PICKUP_STATION})
     * that will dynamically determine which to use based on
     * {@link OperatorPrefs.PickupMode#getCurrentMode()}.
     * the decided proxy command can only be changed by calling
     * {@link #initialize()} again.
     */
    public static class TransitionToPickup extends CommandBase {
        private final SuperStructure superStructure;
        private Command placeCmd;

        public TransitionToPickup(final SuperStructure superStructure) {
            this.superStructure = superStructure;
            addRequirements(superStructure);
        }

        @Override
        public void initialize() {
            switch (OperatorPrefs.PickupMode.getCurrentMode()) {
                case GROUND:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PICKUP_GROUND);
                    break;
                case STATION:
                    placeCmd = new StateManager.CmdTransitionState(
                            superStructure,
                            States.PICKUP_STATION);
                    break;
            }
            placeCmd.initialize();
        }

        @Override
        public void execute() {
            placeCmd.execute();
            // if (superStructure.maybeHasGamepiece()) {
            // }
        }

        @Override
        public void end(boolean interrupted) {
            placeCmd.end(interrupted);
            GamepieceMode.setHeldPiece(GamepieceMode.getDesiredPiece());
        }

        @Override
        public boolean isFinished() {
            return placeCmd.isFinished();
        }

        @Override
        public String getName() {
            if (placeCmd == null)
                return "TransitionToPickup(null)";
            return "TransitionToPickup(" + placeCmd.getName() + ")";
        }
    }
}