package monologue;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;

class MonologueEntryLayer {
  private static final HashMap<LogSink, HashMap<String, MonologueEntry<?>>> entries = new HashMap<>() {
    {
      put(LogSink.NT, new HashMap<>());
      put(LogSink.DL, new HashMap<>());
      put(LogSink.OP, new HashMap<>());
    }
  };

  public static interface MonologueEntry<T> {
    public void log(T value);

    @SuppressWarnings("unchecked")
    public static <T> MonologueEntry<T> create(String path, Class<T> clazz, LogSink sink) {
      var map = entries.get(sink);
      if (!map.containsKey(path)) {
        String cleanPath = NetworkTable.normalizeKey(path, true);
        var e = switch (sink) {
          case NT -> new MonologueNtEntry<>(cleanPath, Optional.empty(), clazz);
          case DL -> new MonologueFileEntry<>(cleanPath, Optional.empty(), clazz);
          case OP -> new MonologueOptimizedEntry<>(cleanPath, Optional.empty(), clazz);
        };
        entries.get(sink).put(path, e);
        return e;
      } else {
        return (MonologueEntry<T>) map.get(path);
      }
    }

    @SuppressWarnings("unchecked")
    public static <T> MonologueEntry<T> create(String path, Struct<T> struct, Class<T> clazz, LogSink sink) {
      var map = entries.get(sink);
      if (!map.containsKey(path)) {
        String cleanPath = NetworkTable.normalizeKey(path, true);
        var e = switch (sink) {
          case NT -> new MonologueNtEntry<>(cleanPath, Optional.of(struct), clazz);
          case DL -> new MonologueFileEntry<>(cleanPath, Optional.of(struct), clazz);
          case OP -> new MonologueOptimizedEntry<>(cleanPath, Optional.of(struct), clazz);
        };
        map.put(path, e);
        return e;
      } else {
        return (MonologueEntry<T>) map.get(path);
      }
    }

    @SuppressWarnings("unchecked")
    public static <T> MonologueEntry<T> createStructArray(String path, Struct<?> struct, Class<T> clazz, LogSink sink) {
      var map = entries.get(sink);
      if (!map.containsKey(path)) {
        String cleanPath = NetworkTable.normalizeKey(path, true);
        var e = switch (sink) {
          case NT -> new MonologueNtEntry<>(cleanPath, Optional.of(struct), clazz);
          case DL -> new MonologueFileEntry<>(cleanPath, Optional.of(struct), clazz);
          case OP -> new MonologueOptimizedEntry<>(cleanPath, Optional.of(struct), clazz);
        };
        map.put(path, e);
        return e;
      } else {
        return (MonologueEntry<T>) map.get(path);
      }
    }
  }

  private static class MonologueFileEntry<T> implements MonologueEntry<T> {
    private final Consumer<T> fileLog;

    @SuppressWarnings("unchecked")
    public MonologueFileEntry(String path, Optional<Struct<?>> optStruct, Class<T> clazz) {
      DataLog dl = DataLogManager.getLog();
      if (optStruct.isPresent()) {
        if (clazz.isArray()) {
          var entry = StructArrayLogEntry.create(dl, path, optStruct.get());
          fileLog = v -> ((StructArrayLogEntry<Object>) entry).append((Object[]) v);
        } else {
          StructLogEntry<T> entry = StructLogEntry.create(dl, path, (Struct<T>) optStruct.get());
          fileLog = entry::append;
        }
      } else if (clazz.equals(Double.class) || clazz.equals(double.class)) {
        DoubleLogEntry entry = new DoubleLogEntry(dl, path);
        fileLog = v -> entry.append((double) v);
      } else if (clazz.equals(Float.class) || clazz.equals(float.class)) {
        FloatLogEntry entry = new FloatLogEntry(dl, path);
        fileLog = v -> entry.append((float) v);
      } else if (clazz.equals(Boolean.class) || clazz.equals(boolean.class)) {
        BooleanLogEntry entry = new BooleanLogEntry(dl, path);
        fileLog = v -> entry.append((boolean) v);
      } else if (clazz.equals(Integer.class) || clazz.equals(int.class)) {
        IntegerLogEntry entry = new IntegerLogEntry(dl, path);
        fileLog = v -> entry.append((int) v);
      } else if (clazz.equals(Long.class) || clazz.equals(long.class)) {
        IntegerLogEntry entry = new IntegerLogEntry(dl, path);
        fileLog = v -> entry.append((long) v);
      } else if (clazz.equals(String.class)) {
        StringLogEntry entry = new StringLogEntry(dl, path);
        fileLog = v -> entry.append((String) v);
      } else if (clazz.equals(Double[].class) || clazz.equals(double[].class)) {
        DoubleArrayLogEntry entry = new DoubleArrayLogEntry(dl, path);
        fileLog = v -> entry.append((double[]) v);
      } else if (clazz.equals(Float[].class) || clazz.equals(float[].class)) {
        FloatArrayLogEntry entry = new FloatArrayLogEntry(dl, path);
        fileLog = v -> entry.append((float[]) v);
      } else if (clazz.equals(Boolean[].class) || clazz.equals(boolean[].class)) {
        BooleanArrayLogEntry entry = new BooleanArrayLogEntry(dl, path);
        fileLog = v -> entry.append((boolean[]) v);
      } else if (clazz.equals(Integer[].class) || clazz.equals(int[].class)) {
        IntegerArrayLogEntry entry = new IntegerArrayLogEntry(dl, path);
        fileLog = v -> {
          int[] ints = (int[]) v;
          long[] longs = new long[ints.length];
          for (int i = 0; i < ints.length; i++) {
            longs[i] = ints[i];
          }
          entry.append(longs);
        };
      } else if (clazz.equals(String[].class)) {
        StringArrayLogEntry entry = new StringArrayLogEntry(dl, path);
        fileLog = v -> entry.append((String[]) v);
      } else if (clazz.equals(byte[].class) || clazz.equals(Byte[].class)) {
        RawLogEntry entry = new RawLogEntry(dl, path);
        fileLog = v -> entry.append((byte[]) v);
      } else {
        throw new IllegalArgumentException("Unsupported type: " + clazz);
      }
    }

    @Override
    public void log(T value) {
      fileLog.accept(value);
    }
  }

  private static class MonologueNtEntry<T> implements MonologueEntry<T> {
    private final Consumer<T> ntLog;

    @SuppressWarnings("unchecked")
    public MonologueNtEntry(String path, Optional<Struct<?>> optStruct, Class<T> clazz) {
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      if (optStruct.isPresent()) {
        if (clazz.isArray()) {
          var entry = nt.getStructArrayTopic(path, optStruct.get()).publish();
          ntLog = v -> ((StructArrayPublisher<Object>) entry).set((Object[]) v);
        } else {
          StructPublisher<T> entry = nt.getStructTopic(path, (Struct<T>) optStruct.get()).publish();
          ntLog = entry::set;
        }
      } else if (clazz.equals(Double.class) || clazz.equals(double.class)) {
        DoublePublisher entry = nt.getDoubleTopic(path).publish();
        ntLog = v -> entry.set((double) v);
      } else if (clazz.equals(Float.class) || clazz.equals(float.class)) {
        FloatPublisher entry = nt.getFloatTopic(path).publish();
        ntLog = v -> entry.set((float) v);
      } else if (clazz.equals(Boolean.class) || clazz.equals(boolean.class)) {
        BooleanPublisher entry = nt.getBooleanTopic(path).publish();
        ntLog = v -> entry.set((boolean) v);
      } else if (clazz.equals(Integer.class) || clazz.equals(int.class)) {
        IntegerPublisher entry = nt.getIntegerTopic(path).publish();
        ntLog = v -> entry.set((int) v);
      } else if (clazz.equals(Long.class) || clazz.equals(long.class)) {
        IntegerPublisher entry = nt.getIntegerTopic(path).publish();
        ntLog = v -> entry.set((long) v);
      } else if (clazz.equals(String.class)) {
        StringPublisher entry = nt.getStringTopic(path).publish();
        ntLog = v -> entry.set((String) v);
      } else if (clazz.equals(Double[].class) || clazz.equals(double[].class)) {
        DoubleArrayPublisher entry = nt.getDoubleArrayTopic(path).publish();
        ntLog = v -> entry.set((double[]) v);
      } else if (clazz.equals(Float[].class) || clazz.equals(float[].class)) {
        FloatArrayPublisher entry = nt.getFloatArrayTopic(path).publish();
        ntLog = v -> entry.set((float[]) v);
      } else if (clazz.equals(Boolean[].class) || clazz.equals(boolean[].class)) {
        BooleanArrayPublisher entry = nt.getBooleanArrayTopic(path).publish();
        ntLog = v -> entry.set((boolean[]) v);
      } else if (clazz.equals(Integer[].class) || clazz.equals(int[].class)) {
        IntegerArrayPublisher entry = nt.getIntegerArrayTopic(path).publish();
        ntLog = v -> {
          int[] ints = (int[]) v;
          long[] longs = new long[ints.length];
          for (int i = 0; i < ints.length; i++) {
            longs[i] = ints[i];
          }
          entry.set(longs);
        };
      } else if (clazz.equals(String[].class)) {
        StringArrayPublisher entry = nt.getStringArrayTopic(path).publish();
        ntLog = v -> entry.set((String[]) v);
      } else if (clazz.equals(byte[].class) || clazz.equals(Byte[].class)) {
        RawPublisher entry = nt.getRawTopic(path).publish("raw");
        ntLog = v -> entry.set((byte[]) v);
      } else {
        throw new IllegalArgumentException("Unsupported type: " + clazz);
      }
    }

    @Override
    public void log(T value) {
      ntLog.accept(value);
    }
  }

  private static class MonologueOptimizedEntry<T> implements MonologueEntry<T> {
    private final MonologueFileEntry<T> fileEntry;
    private final MonologueNtEntry<T> ntEntry;

    public MonologueOptimizedEntry(String path, Optional<Struct<?>> optStruct, Class<T> clazz) {
      fileEntry = new MonologueFileEntry<>(path, optStruct, clazz);
      ntEntry = new MonologueNtEntry<>(path, optStruct, clazz);
    }

    @Override
    public void log(T value) {
      if (Monologue.isBandwidthOptimizationEnabled()) {
        fileEntry.log(value);
      } else {
        ntEntry.log(value);
      }
    }
  }
}
