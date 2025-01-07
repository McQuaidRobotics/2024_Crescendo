package igknighters.util.can;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Timer;
import igknighters.util.plumbing.Channel;
import igknighters.util.plumbing.Channel.Receiver;
import igknighters.util.plumbing.Channel.Sender;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import monologue.TimeSensitiveLogger.TimestampedDouble;

public class StatusSignalQueuer {
  private final AtomicBoolean running = new AtomicBoolean();
  private final double frequency;
  private final AtomicLong measuredFrequency = new AtomicLong();
  private final Thread thread;
  private final BaseStatusSignal[] signals;
  private final double[] previousFrequencies;
  private final Sender<TimestampedDouble>[] senders;
  private final Receiver<TimestampedDouble>[] receivers;

  @SuppressWarnings("unchecked")
  public StatusSignalQueuer(int frequency, StatusSignal<?>... signals) {
    this.frequency = frequency;
    this.signals = new BaseStatusSignal[signals.length];
    this.previousFrequencies = new double[signals.length];
    final List<Sender<TimestampedDouble>> senders = new ArrayList<>();
    final List<Receiver<TimestampedDouble>> receivers = new ArrayList<>();
    for (int i = 0; i < signals.length; i++) {
      previousFrequencies[i] = signals[i].getAppliedUpdateFrequency();
      this.signals[i] = signals[i].clone();
      final Channel<TimestampedDouble> channel = new Channel<>(new TimestampedDouble[0]);
      senders.add(channel.sender());
      receivers.add(channel.openReceiver(64, Channel.ThreadSafetyMarker.CONCURRENT));
    }
    this.senders = senders.toArray(new Sender[0]);
    this.receivers = receivers.toArray(new Receiver[0]);
    this.thread = new Thread(this::run);
  }

  public Receiver<TimestampedDouble>[] receivers() {
    return receivers;
  }

  public void start() {
    thread.start();
    for (int i = 0; i < signals.length; i++) {
      previousFrequencies[i] = signals[i].getAppliedUpdateFrequency();
    }
    BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);
    running.set(true);
  }

  public void stop() {
    if (!running.get()) {
      return;
    }
    thread.interrupt();
    for (int i = 0; i < signals.length; i++) {
      signals[i].setUpdateFrequency(previousFrequencies[i]);
    }
    running.set(false);
  }

  public double getMeasuredFrequency() {
    return Double.longBitsToDouble(measuredFrequency.get());
  }

  public boolean isRunning() {
    return running.get();
  }

  public void setRunning(boolean running) {
    if (running) {
      start();
    } else {
      stop();
    }
  }

  private void run() {
    final Timer timer = new Timer();
    timer.start();
    double count = 0;
    final double timeout = (1.0 / frequency) * 1.5;
    while (!Thread.interrupted()) {
      BaseStatusSignal.waitForAll(timeout, signals);
      for (int i = 0; i < signals.length; i++) {
        final TimestampedDouble value =
            new TimestampedDouble(
                signals[i].getValueAsDouble(),
                Timer.getFPGATimestamp() - signals[i].getTimestamp().getLatency());
        senders[i].send(value);
      }
      final double elapsed = timer.get();
      timer.reset();
      count++;
      measuredFrequency.set(Double.doubleToLongBits(count / elapsed));
    }
  }
}
