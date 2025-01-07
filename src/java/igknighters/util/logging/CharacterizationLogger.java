package igknighters.util.logging;

import com.ctre.phoenix6.StatusSignal;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kCharacterization.MechanismId;
import igknighters.util.can.StatusSignalQueuer;
import igknighters.util.plumbing.Channel.Receiver;
import java.util.Optional;
import monologue.TimeSensitiveLogger;
import monologue.TimeSensitiveLogger.TimestampedDouble;

public class CharacterizationLogger {
  private static final String table = "/Characterization/";

  private Optional<MechanismId> activeMechanism = Optional.empty();

  private final MechanismId mechanism;
  private final TimeSensitiveLogger[] loggers;
  private final Receiver<TimestampedDouble>[] receivers;
  private final StatusSignalQueuer queuer;

  public CharacterizationLogger(MechanismId mechanism, StatusSignal<?>... signals) {
    this.mechanism = mechanism;
    loggers = new TimeSensitiveLogger[signals.length];
    for (int i = 0; i < signals.length; i++) {
      loggers[i] = new TimeSensitiveLogger(table + mechanism + "/" + signals[i].getName());
    }
    queuer = new StatusSignalQueuer(ConstValues.kCharacterization.FREQUENCY, signals);
    receivers = queuer.receivers();
  }

  public void log() {
    if (activeMechanism.isEmpty()) return;
    queuer.setRunning(activeMechanism.get() == mechanism);
    if (!queuer.isRunning()) return;
    for (int i = 0; i < loggers.length; i++) {
      loggers[i].log(receivers[i].recvAll());
    }
  }
}
