package igknighters.util.plumbing;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * A utility implementing broadcast(multi-sender, multi-receiver) type-safe channels for inner and
 * inter thread communication
 */
public class Channel<T> {
  private final T[] templateArray;
  private final ArrayList<WeakReference<Receiver<T>>> receivers = new ArrayList<>();
  private final Sender<T> sender = new Sender<>(this);

  public Channel(T[] template) {
    templateArray = template;
  }

  /**
   * Returns the sender for this channel
   *
   * @return the sender for this channel
   */
  public Sender<T> sender() {
    return sender;
  }

  /**
   * Opens a new {@link Receiver} for this channel.
   *
   * @param bufferSize the size of the buffer for the receiver
   * @return a new receiver for this channel
   */
  public Receiver<T> openReceiver(int bufferSize) {
    Receiver<T> r = new ReceiverBuffered<>(this, makeBuffer(bufferSize), false);
    addReceiver(r);
    return r;
  }

  /**
   * Opens a new {@link Receiver} for this channel.
   *
   * @param bufferSize the size of the buffer for the receiver
   * @param safetyMarker the safety marker for the receiver
   * @return a new receiver for this channel
   */
  @SuppressWarnings("unchecked")
  public <TSM extends ThreadSafetyMarker> SafetyMarkedReceiver<T, TSM> openReceiver(
      int bufferSize, TSM safetyMarker) {
    // this allows the type system to enforce our thread safety without needing to define a new
    // class for each safety marker
    var r =
        (SafetyMarkedReceiver<T, TSM>)
            new ReceiverBuffered<>(this, makeBuffer(bufferSize), safetyMarker.threadSafe());
    addReceiver(r);
    return r;
  }

  private void popReceiver(Receiver<T> receiver) {
    for (int i = 0; i < receivers.size(); i++) {
      if (receivers.get(i).get() == receiver) {
        receivers.remove(i);
      }
    }
  }

  private void addReceiver(Receiver<T> receiver) {
    for (int i = 0; i < receivers.size(); i++) {
      if (receivers.get(i).get() == receiver) {
        return;
      }
    }
    receivers.add(new WeakReference<>(receiver));
  }

  T[] makeBuffer(int size) {
    return Arrays.copyOf(templateArray, size);
  }

  /**
   * Sends data to the channel, multicasting it to all receivers
   *
   * @param newValue the new value to send
   */
  public void push(T newValue) {
    for (WeakReference<Receiver<T>> receiver : receivers) {
      Receiver<T> r = receiver.get();
      if (r != null) {
        r.push(newValue);
      } else {
        receivers.remove(receiver);
      }
    }
  }

  public interface ThreadSafetyMarker {
    public static final Concurrent CONCURRENT = new Concurrent();
    public static final Sequential SEQUENTIAL = new Sequential();

    boolean threadSafe();
  }

  private static class Internal implements ThreadSafetyMarker {
    @Override
    public boolean threadSafe() {
      throw new UnsupportedOperationException("Internal safety marker should not be used");
    }
  }

  public static class Concurrent implements ThreadSafetyMarker {
    @Override
    public boolean threadSafe() {
      return true;
    }
  }

  public static class Sequential implements ThreadSafetyMarker {
    @Override
    public boolean threadSafe() {
      return false;
    }
  }

  /** The sending end of a broadcast channel, the sent value will be multicast to all receivers */
  public static final class Sender<T> implements Consumer<T> {
    private final WeakReference<Channel<T>> channel;

    private Sender(Channel<T> channel) {
      this.channel = new WeakReference<>(channel);
    }

    @Override
    public void accept(T t) {
      send(t);
    }

    /**
     * Sends data to the channel
     *
     * @param newValue the new value to send
     */
    public void send(T newValue) {
      var c = channel.get();
      if (c != null) {
        c.push(newValue);
      }
    }
  }

  /**
   * The receiving end of a channel, data can be extracted from it, specific behavior depends on the
   * chosen channel type.
   *
   * @param <T> The type of the channel
   */
  public abstract static class Receiver<T> {
    private final Channel<T> channel;

    Receiver(Channel<T> channel) {
      this.channel = channel;
    }

    /**
     * Will push the new value to the channel's receivers
     *
     * @param newValue the new value to push
     */
    protected abstract void push(T newValue);

    /**
     * @return the received value and removes it from the channel
     */
    public abstract T recv();

    /**
     * Inspects the received value without removing it from the channel
     *
     * @return the received value
     */
    public abstract T inspect();

    /**
     * @return whether the channel has any content
     */
    public abstract boolean hasData();

    /**
     * @return all received values, removing them from the channel
     */
    public abstract T[] recvAll();

    /**
     * @return an optional containing the received value, or empty if the channel is empty, removing
     *     the value from the channel
     */
    public Optional<T> tryRecv() {
      if (hasData()) {
        return Optional.of(recv());
      } else {
        return Optional.empty();
      }
    }

    /**
     * Attempts to inspect the received value without removing it from the channel
     *
     * @return an optional containing the received value, or empty if the channel is
     */
    public Optional<T> tryInspect() {
      if (hasData()) {
        return Optional.of(inspect());
      } else {
        return Optional.empty();
      }
    }

    /**
     * @return the received value or the default value if the channel is empty, removing the value
     *     from the channel
     */
    public T recvOrDefault(T defaultValue) {
      if (hasData()) {
        return recv();
      } else {
        return defaultValue;
      }
    }

    /**
     * @return the received value or the default value if the channel is empty, without removing the
     *     value from the channel
     */
    public T inspectOrDefault(T defaultValue) {
      if (hasData()) {
        return inspect();
      } else {
        return defaultValue;
      }
    }

    /** Closes the receiver, removing it from the channel */
    public void close() {
      channel.popReceiver(this);
    }

    /** Re-opens the receiver, adding it back to the channel */
    public void open() {
      channel.addReceiver(this);
    }

    /**
     * Forks the receiver, creating a new receiver that will receive the same data.
     *
     * @param bufferSize the size of the buffer for the new receiver
     * @return the forked receiver
     */
    public Receiver<T> fork(int bufferSize) {
      Receiver<T> r = new ReceiverBuffered<>(channel, channel.makeBuffer(bufferSize), false);
      channel.addReceiver(r);
      return r;
    }

    /**
     * Forks the receiver, creating a new receiver that will receive the same data.
     *
     * @param bufferSize the size of the buffer for the new receiver
     * @param safetyMarker the safety marker for the new receiver
     * @return the forked receiver
     */
    @SuppressWarnings("unchecked")
    public <TSM extends ThreadSafetyMarker> SafetyMarkedReceiver<T, TSM> fork(
        int bufferSize, TSM safetyMarker) {
      var r =
          (SafetyMarkedReceiver<T, TSM>)
              new ReceiverBuffered<>(
                  channel, channel.makeBuffer(bufferSize), safetyMarker.threadSafe());
      channel.addReceiver(r);
      return r;
    }
  }

  public abstract static class SafetyMarkedReceiver<T, TSM extends ThreadSafetyMarker>
      extends Receiver<T> {
    SafetyMarkedReceiver(Channel<T> channel) {
      super(channel);
    }
  }

  /**
   * The receiving end of a broadcast channel that stores all values in a circular buffer, can hold
   * up to specified number of values
   */
  private static final class ReceiverBuffered<T> extends SafetyMarkedReceiver<T, Internal> {
    private final Optional<Lock> lock;

    /** the buffer of all currently held values */
    private final T[] buffer;

    /** the index of the newest data */
    private int newestIndex = 0;

    /** the index of the oldest data */
    private int oldestIndex = 0;

    public ReceiverBuffered(Channel<T> channel, T[] buffer, boolean threadSafe) {
      super(channel);
      lock = threadSafe ? Optional.of(new ReentrantLock(false)) : Optional.empty();
      this.buffer = buffer;
    }

    @Override
    public T recv() {
      lock.ifPresent(Lock::lock);
      try {
        T value = buffer[oldestIndex];
        buffer[oldestIndex] = null;
        oldestIndex = (oldestIndex + 1) % buffer.length;
        return value;
      } finally {
        lock.ifPresent(Lock::unlock);
      }
    }

    @Override
    public T inspect() {
      lock.ifPresent(Lock::lock);
      try {
        return buffer[oldestIndex];
      } finally {
        lock.ifPresent(Lock::unlock);
      }
    }

    @Override
    public boolean hasData() {
      lock.ifPresent(Lock::lock);
      try {
        return buffer[oldestIndex] != null;
      } finally {
        lock.ifPresent(Lock::unlock);
      }
    }

    @Override
    public T[] recvAll() {
      lock.ifPresent(Lock::lock);
      try {
        if (oldestIndex == newestIndex) {
          return Arrays.copyOf(buffer, 0);
        } else if (oldestIndex < newestIndex) {
          return Arrays.copyOfRange(buffer, oldestIndex, newestIndex);
        } else {
          T[] tail = Arrays.copyOfRange(buffer, oldestIndex, buffer.length);
          T[] head = Arrays.copyOfRange(buffer, 0, newestIndex);
          T[] result = Arrays.copyOf(tail, tail.length + head.length);
          System.arraycopy(head, 0, result, tail.length, head.length);
          return head;
        }
      } finally {
        newestIndex = 0;
        oldestIndex = 0;
        Arrays.fill(buffer, null);
        lock.ifPresent(Lock::unlock);
      }
    }

    @Override
    protected void push(T newValue) {
      lock.ifPresent(Lock::lock);
      try {
        buffer[newestIndex] = newValue;
        newestIndex = (newestIndex + 1) % buffer.length;
        if (newestIndex == oldestIndex) {
          oldestIndex = (oldestIndex + 1) % buffer.length;
        }
      } finally {
        lock.ifPresent(Lock::unlock);
      }
    }
  }
}
