package monologue;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

class RuntimeLog {
  private static final StringPublisher entry;

  static {
    // we need to make sure we never log network tables through the implicit wpilib logger
    entry = NetworkTableInstance.getDefault().getStringTopic("/MonologueSetup").publish();
    info("Monologue Setup Logger initialized");
  }

  private static class MonologueRuntimeError extends RuntimeException {
    MonologueRuntimeError(String message) {
      super(message);
    }
  }

  public static void info(String message) {
    entry.set("[Monologue] " + message);
  }

  public static void warn(String warning) {
    if (Monologue.shouldThrowOnWarn()) {
      throw new MonologueRuntimeError("[Monologue] " + warning);
    } else {
      String message = "[Monologue] (WARNING) " + warning;
      entry.set(message);
      DriverStationJNI.sendError(false, 1, false, message, "", "", true);
    }
  }
}
