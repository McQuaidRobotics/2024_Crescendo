package frc.robot.commands.superstructure;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.super_structure.States;
import frc.robot.subsystems.super_structure.SuperStructure;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;

/**
 * Used to define State->State transition commands, to be used by
 * {@link StateManager}
 */
public class Transitions {

    public static class TransitionData {
        public final States from, to;
        public final SuperStructure superStructure;

        public TransitionData(States from, States to, SuperStructure superStructure) {
            this.from = from;
            this.to = to;
            this.superStructure = superStructure;
        }
    }

    public static Command defaultTransition(TransitionData data) {
        return data.superStructure.run(() -> {
            data.superStructure.setSetpoint(
                    SuperStructurePosition.fromState(data.to));
        });
    }

    public static Command homeTransition(TransitionData data) {
        return new FunctionalCommand(
            //do a force home on start incase its rescheduled it will try homing again
            () -> data.superStructure.home(true),
            () -> data.superStructure.home(false),
            (bool) -> OperatorPrefs.NEED_HOME = false,
            () -> data.superStructure.isHomed(),
            data.superStructure
        //then run a normal stow to hold the mechanism in place
        ).andThen(stowTransition(data));
    }

    public static Command stowTransition(TransitionData data) {
        return new Command() {
            private Integer cycles = 0;;

            @Override
            public void execute() {
                if (data.superStructure.setSetpoint(SuperStructurePosition.fromState(data.to))) {
                    cycles++;
                }
                if (cycles > 50) {
                    var ampInfo = data.superStructure.getComponentAmps();
                    for (var compAmp : ampInfo) {
                        if (compAmp > 20) {
                            OperatorPrefs.NEED_HOME = true;
                            return;
                        }
                    }
                }
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(data.superStructure);
            }
        };
    }

    public static Command placeHighTransition(TransitionData data) {
        return data.superStructure.run(() -> {
            SuperStructurePosition toPose;
            if (data.superStructure.getPose().elevatorMeters < States.PLACE_MID.elevatorMeters) {
                var wristOffset = (States.STOW.wristDegrees - States.PLACE_HIGH.wristDegrees)/2.0;
                toPose = new SuperStructurePosition(
                    data.to.wristDegrees + wristOffset,
                    data.to.pivotDegrees,
                    data.to.elevatorMeters,
                    0.0
                );
            } else {
                toPose = SuperStructurePosition.fromState(data.to);
            }
            data.superStructure.setSetpoint(toPose);
        });
    }

    public static Command placeLowAntiChopTransition(TransitionData data) {
        return data.superStructure.run(() -> {
            SuperStructurePosition toPose;
            if (data.superStructure.getPose().pivotDegrees < States.PLACE_LOW_FRONT.pivotDegrees * 0.95) {
                toPose = new SuperStructurePosition(
                    data.to.wristDegrees + 20.0,
                    data.to.pivotDegrees,
                    data.to.elevatorMeters,
                    0.0
                );
            } else {
                toPose = SuperStructurePosition.fromState(data.to);
            }
            data.superStructure.setSetpoint(toPose);
        });
    }
}
