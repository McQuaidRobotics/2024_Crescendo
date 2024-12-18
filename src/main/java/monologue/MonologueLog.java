package monologue;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

class MonologueLog {
  private static final StringPublisher entry;

  static {
    // we need to make sure we never log network tables through the implicit wpilib logger
    entry = NetworkTableInstance.getDefault().getStringTopic("/MonologueSetup").publish();
    runtimeLog("Monologue Setup Logger initialized");
  }

  private static class MonologueRuntimeError extends RuntimeException {
    MonologueRuntimeError(String message) {
      super(message);
    }
  }

  static void runtimeLog(String message) {
    entry.set("[Monologue] " + message);
  }

  static void runtimeWarn(String warning) {
    if (Monologue.shouldThrowOnWarn()) {
      throw new MonologueRuntimeError("[Monologue] " + warning);
    } else {
      String message = "[Monologue] (WARNING) " + warning;
      entry.set(message);
      DriverStationJNI.sendError(false, 1, false, message, "", "", true);
    }
  }
}
