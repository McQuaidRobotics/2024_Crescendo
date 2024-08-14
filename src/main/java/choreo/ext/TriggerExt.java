package choreo.ext;

import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerExt extends Trigger {
    private static final VarHandle loopHandle;

    static {
        try {
            loopHandle = MethodHandles.privateLookupIn(Trigger.class, MethodHandles.lookup())
                    .findVarHandle(Trigger.class, "m_loop", EventLoop.class);
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

    public static TriggerExt from(Trigger trigger) {
        if (trigger instanceof TriggerExt) {
            return (TriggerExt) trigger;
        } else {
            return new TriggerExt(
                (EventLoop) loopHandle.get(trigger),
                trigger
            );
        }
    }
}
