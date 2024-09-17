package choreo.ext;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerExt extends Trigger {
    private static final VarHandle loopHandle;
    private static final VarHandle conditionHandle;

    static {
        try {
            loopHandle = MethodHandles.privateLookupIn(Trigger.class, MethodHandles.lookup())
                    .findVarHandle(Trigger.class, "m_loop", EventLoop.class);
            conditionHandle = MethodHandles.privateLookupIn(Trigger.class, MethodHandles.lookup())
                    .findVarHandle(Trigger.class, "m_condition", BooleanSupplier.class);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    public TriggerExt(EventLoop loop, BooleanSupplier condition) {
        super(loop, condition);
    }

    public TriggerExt onTrueWith(BooleanSupplier andCondition, Command andTrueCmd, Command andFalseCmd) {
        this.and(andCondition).onTrue(andTrueCmd);
        this.and(() -> !andCondition.getAsBoolean()).onTrue(andFalseCmd);
        return this;
    }

    /**
    * Sets up a {@link Command} to mimic a default command while a condition is true.
    *
    * <p>The command will not interrupt any command other than the original default command of the
    * subsystems the command requires.
    *
    * @param command the command to start
    * @return this trigger, so calls can be chained
    */
    public TriggerExt whileTrueDefault(Command cmd) {
        // you could implement this by overiding the subsystems default command
        // but that has alot of foot guns and likely would leak into causing issues
        BooleanSupplier cond = (BooleanSupplier) conditionHandle.get(this);
        ((EventLoop) loopHandle.get(this))
                .bind(
                        new Runnable() {
                            private final CommandScheduler scheduler = CommandScheduler.getInstance();

                            public boolean freeToScehdule(Command cmd) {
                                var requirements = cmd.getRequirements();
                                for (var requirement : requirements) {
                                    // todo test this logic better for null cases
                                    if (scheduler.requiring(requirement) != requirement.getDefaultCommand()) {
                                        return false;
                                    }
                                }
                                return true;
                            }

                            @Override
                            public void run() {
                                boolean pressed = cond.getAsBoolean();

                                if (pressed) {
                                    if (!cmd.isScheduled() && freeToScehdule(cmd)) {
                                        cmd.schedule();
                                    }
                                } else if (!pressed && cmd.isScheduled()) {
                                    cmd.cancel();
                                }
                            }
                        });
        return this;
    }

    public static TriggerExt from(Trigger trigger) {
        if (trigger instanceof TriggerExt) {
            return (TriggerExt) trigger;
        } else {
            return new TriggerExt(
                    (EventLoop) loopHandle.get(trigger),
                    trigger);
        }
    }
}
