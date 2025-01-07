package igknighters.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedAnimations;
import java.util.concurrent.atomic.AtomicInteger;

public class LedCommands {
  public static Command animate(Led led, LedAnimations pattern) {
    return animate(led, pattern, 9999.0);
  }

  public static Command animate(Led led, LedAnimations pattern, double timeout) {
    AtomicInteger handle = new AtomicInteger();
    return Commands.runOnce(() -> handle.set(led.reserve()))
        .andThen(() -> led.animate(handle.get(), pattern))
        .finallyDo(() -> led.release(handle.get()))
        .withTimeout(timeout)
        .withName("LedAnimate[" + pattern + "]");
  }
}
