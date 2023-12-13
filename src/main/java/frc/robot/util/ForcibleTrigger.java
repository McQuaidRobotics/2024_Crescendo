package frc.robot.util;

import java.util.function.BooleanSupplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ForcibleTrigger extends Trigger {

    private final BooleanSupplier condition;
    private final EventLoop loop;

    public ForcibleTrigger(BooleanSupplier condition) {
        super(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
        this.condition = condition;
        loop = CommandScheduler.getInstance().getDefaultButtonLoop();
    }


    /**
     * Starts the given command whenever the condition changes from `false` to
     * `true`. Will cancel any already-running command.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public Trigger onTrueForce(Command command) {
        requireNonNullParam(command, "command", "onRising");
        loop.bind(
                new Runnable() {
                    private boolean m_pressedLast = condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            var scheduler = CommandScheduler.getInstance();
                            if (scheduler.isScheduled(command)) {
                                scheduler.cancel(command);
                            }
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    public static ForcibleTrigger from(BooleanSupplier condition) {
        return new ForcibleTrigger(condition);
    }
}
