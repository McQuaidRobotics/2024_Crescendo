package monologue;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class TimeSensitiveLogger {
  public record TimestampedDouble(double value, double timestamp) {}

  private final DoubleLogEntry logEntry;

  public TimeSensitiveLogger(String key) {
    logEntry = new DoubleLogEntry(DataLogManager.getLog(), key);
  }

  private long secondsToMicros(double seconds) {
    return (long) (seconds * 1_000_000);
  }

  public void log(double value, double timestampSeconds) {
    logEntry.append(value, secondsToMicros(timestampSeconds));
  }

  public void log(TimestampedDouble value) {
    log(value.value, value.timestamp);
  }

  public void log(TimestampedDouble... values) {
    for (TimestampedDouble value : values) {
      log(value);
    }
  }
}
