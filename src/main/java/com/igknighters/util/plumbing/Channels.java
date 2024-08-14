package com.igknighters.util.plumbing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;
import java.lang.reflect.Type;

/**
 * A utility implementing broadcast(multi-sender, multi-receiver) type-safe channels for inner and inter thread communication
 */
public class Channels {
    private static final HashMap<String, ArrayList<Receiver<?>>> receivers = new HashMap<>();
    private static final HashMap<String, Type> types = new HashMap<>();

    /**
     * The receiving end of a channel, data can be extracted from it,
     * specific behavior depends on the chosen channel type.
     * 
     * @param <T> The type of the channel
     */
    public static abstract class Receiver<T> {
        Receiver(Class<T> clazz, String channelName) {
            if (!types.containsKey(channelName)) {
                types.put(channelName, clazz);
            } else {
                if (types.get(channelName) != clazz) {
                    throw new RuntimeException("Channel " + channelName + " already exists with type "
                            + types.get(channelName) + " but tried to create with type " + clazz);
                }
            }
            if (!receivers.containsKey(channelName)) {
                receivers.put(channelName, new ArrayList<>());
            }
            receivers.get(channelName).add(this);
        }

        /**
         * Will send data to the receiver
         * 
         * @param newValue the new value to send
         */
        protected void send(T newValue) {
        }

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
         * @return an optional containing the received value, or empty if the channel is
         *         empty,
         *         removing the value from the channel
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
         * @return the received value or the default value if the channel is empty,
         *         removing the value from the channel
         */
        public T recvOrDefault(T defaultValue) {
            if (hasData()) {
                return recv();
            } else {
                return defaultValue;
            }
        }

        /**
         * @return the received value or the default value if the channel is empty,
         *         without removing the value from the channel
         */
        public T inspectOrDefault(T defaultValue) {
            if (hasData()) {
                return inspect();
            } else {
                return defaultValue;
            }
        }

        /**
         * @return whether the channel has any content
         */
        public boolean hasData() {
            return false;
        }

        /**
         * @return the name of the channel
         */
        public abstract String getChannelName();

        /**
         * Creates a receiver that stores the latest value
         * 
         * @param <T>          the type of the channel
         * @param initialValue the initial value of the channel
         * @param channelName  the name of the channel to receive from
         * @return the receiver
         */
        public static <T> Receiver<T> latest(final String channelName, Class<T> clazz) {
            return new ReceiverLatest<T>(clazz, channelName);
        }

        /**
         * Creates a buffered receiver with the specified buffer size
         * 
         * @param <T>         the type of the channel
         * @param bufferSize  the size of the buffer
         * @param channelName the name of the channel to receive from
         * @return the receiver
         */
        public static <T> Receiver<T> buffered(final String channelName, int bufferSize, Class<T> clazz) {
            return new ReceiverBuffered<T>(bufferSize, clazz, channelName);
        }

        /**
         * Creates a buffered receiver with a buffer size of 64
         * 
         * @param <T>         the type of the channel
         * @param channelName the name of the channel to receive from
         * @return the receiver
         */
        public static <T> Receiver<T> buffered(final String channelName, Class<T> clazz) {
            return new ReceiverBuffered<T>(64, clazz, channelName);
        }

        /**
         * Creates a receiver that calls the specified consumer with the received value
         * 
         * @param <T>         the type of the channel
         * @param channelName the name of the channel to receive from
         * @param consumer    the consumer to call with the received value
         * @return the receiver
         */
        public static <T> void reactor(final String channelName, Class<T> clazz, Consumer<T> consumer) {
            new ReceiverReactor<T>(clazz, channelName, consumer);
        }
    }

    /**
     * The receiving end of a broadcast channel that only stores the latest value
     */
    private static final class ReceiverLatest<T> extends Receiver<T> {
        private final ReadWriteLock lock = new ReentrantReadWriteLock();
        private final String channelName;

        private Optional<T> value = Optional.empty();

        public ReceiverLatest(Class<T> clazz, final String channelName) {
            super(clazz, channelName);
            this.channelName = channelName;
        }

        @Override
        public T recv() {
            lock.readLock().lock();
            try {
                return value.get();
            } finally {
                value = Optional.empty();
                lock.readLock().unlock();
            }
        }

        @Override
        public T inspect() {
            lock.readLock().lock();
            try {
                return value.get();
            } finally {
                lock.readLock().unlock();
            }
        }

        @Override
        public boolean hasData() {
            lock.readLock().lock();
            try {
                return value.isPresent();
            } finally {
                lock.readLock().unlock();
            }
        }

        @Override
        protected void send(T newValue) {
            lock.writeLock().lock();
            try {
                value = Optional.of(newValue);
            } finally {
                lock.writeLock().unlock();
            }
        }

        @Override
        public String getChannelName() {
            return channelName;
        }
    }

    /**
     * The receiving end of a broadcast channel that stores all values in a circular
     * buffer,
     * can hold up to specified number of values
     */
    private static final class ReceiverBuffered<T> extends Receiver<T> {
        private final ReadWriteLock lock = new ReentrantReadWriteLock();
        private final String channelName;

        private final T[] buffer;
        /** the index of the newest data */
        private int newIndex = 0;
        /** the index of the oldest data */
        private int oldIndex = 0;

        @SuppressWarnings("unchecked")
        public ReceiverBuffered(int bufferSize, Class<T> clazz, final String channelName) {
            super(clazz, channelName);
            if (bufferSize <= 0) {
                throw new IllegalArgumentException("bufferSize must be positive and non-zero");
            }
            this.channelName = channelName;
            buffer = (T[]) new Object[bufferSize];
            for (int i = 0; i < bufferSize; i++) {
                buffer[i] = null;
            }
        }

        @Override
        public T recv() {
            lock.readLock().lock();
            try {
                T value = buffer[oldIndex];
                buffer[oldIndex] = null;
                oldIndex = (oldIndex + 1) % buffer.length;
                return value;
            } finally {
                lock.readLock().unlock();
            }
        }

        @Override
        public T inspect() {
            lock.readLock().lock();
            try {
                return buffer[oldIndex];
            } finally {
                lock.readLock().unlock();
            }
        }

        @Override
        public boolean hasData() {
            lock.readLock().lock();
            try {
                return buffer[oldIndex] != null;
            } finally {
                lock.readLock().unlock();
            }
        }

        @Override
        protected void send(T newValue) {
            lock.writeLock().lock();
            try {
                buffer[newIndex] = newValue;
                newIndex = (newIndex + 1) % buffer.length;
                if (newIndex == oldIndex) {
                    oldIndex = (oldIndex + 1) % buffer.length;
                }
            } finally {
                lock.writeLock().unlock();
            }
        }

        @Override
        public String getChannelName() {
            return channelName;
        }
    }

    /**
     * The receiving end of a broadcast channel that calls a consumer with the
     * received value
     */
    private static final class ReceiverReactor<T> extends Receiver<T> {
        private final String channelName;
        private final Consumer<T> consumer;

        public ReceiverReactor(Class<T> clazz, final String channelName, Consumer<T> consumer) {
            super(clazz, channelName);
            this.channelName = channelName;
            this.consumer = consumer;
        }

        @Override
        public T recv() {
            throw new UnsupportedOperationException("Reactor channels do not support recv");
        }

        @Override
        public T inspect() {
            throw new UnsupportedOperationException("Reactor channels do not support inspect");
        }

        @Override
        public boolean hasData() {
            return false;
        }

        @Override
        protected void send(T newValue) {
            consumer.accept(newValue);
        }

        @Override
        public String getChannelName() {
            return channelName;
        }
    }


    /**
     * The sending end of a broadcast channel, the sent value will be multicast to
     * all receivers
     */
    public static final class Sender<T> {
        private final String channelName;

        private Sender(final String channelName, Class<T> clazz) {
            this.channelName = channelName;
            if (!receivers.containsKey(channelName)) {
                receivers.put(channelName, new ArrayList<>());
            }
            if (types.containsKey(channelName)) {
                if (types.get(channelName) != clazz) {
                    throw new IllegalArgumentException(
                            "Channel " + channelName + " already exists with type " + types.get(channelName));
                }
            } else {
                types.put(channelName, clazz);
            }
        }

        /**
         * Sends data to the channel
         * 
         * @param newValue the new value to send
         */
        @SuppressWarnings("unchecked")
        public void send(T newValue) {
            for (Receiver<?> receiver : receivers.get(channelName)) {
                ((Receiver<T>) receiver).send(newValue);
            }
        }

        public static <T> Sender<T> broadcast(final String channelName, Class<T> clazz) {
            return new Sender<>(channelName, clazz);
        }
    }
}
