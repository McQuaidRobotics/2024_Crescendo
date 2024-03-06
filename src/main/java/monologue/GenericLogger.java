package monologue;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.*;

import java.util.Collection;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.lang.reflect.Field;

@SuppressWarnings("unchecked")
abstract class GenericLogger {
    public interface LogRunnable extends LongConsumer {
        String key();
        void close();
        void open();
    }
    protected GenericLogger() {}
    private final Map<String, Struct<?>> structTypeCache = new HashMap<>();

    protected void addField(String key, LogLevel level, LongConsumer run, Runnable open, Runnable close) {
        var runnable =  new LogRunnable() {
            @Override
            public String key() {
                return key;
            }
            @Override
            public void accept(long timestamp) {
                try {
                run.accept(timestamp);
                } catch (Exception e) {}
            }
            @Override
            public void open() {
                try {
                open.run();
                } catch (Exception e) {}
            }
            @Override
            public void close() {
                try {
                close.run();
                } catch (Exception e) {}
            }
        };
        runnableMap.get(level).add(runnable);
    }

    protected void addField(String key, LogLevel level, LongConsumer run) {
        addField(key, level, run, ()->{}, ()->{});
    }

    private final Map<LogLevel, ArrayList<LogRunnable>> runnableMap = new HashMap<>(
        Map.of(
            LogLevel.OVERRIDE_FILE_ONLY, new ArrayList<>(),
            LogLevel.DEFAULT, new ArrayList<>(),
            LogLevel.NOT_FILE_ONLY, new ArrayList<>()
        )
    );
    protected final Collection<SendableBuilder> sendables = new LinkedHashSet<>();


    public void put(String entryName, boolean value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, boolean value, LogLevel level) {}

    public void addBoolean(String entryName, Supplier<Boolean> valueSupplier, LogLevel level) {}
    public void addBoolean(String entryName, Supplier<Boolean> valueSupplier) {
        addBoolean(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, int value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, int value, LogLevel level) {}

    public void addInteger(String entryName, Supplier<Integer> valueSupplier, LogLevel level) {}
    public void addInteger(String entryName, Supplier<Integer> valueSupplier) {
        addInteger(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, long value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, long value, LogLevel level) {}

    public void addLong(String entryName, Supplier<Long> valueSupplier, LogLevel level) {}
    public void addLong(String entryName, Supplier<Long> valueSupplier) {
        addLong(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, float value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, float value, LogLevel level) {}

    public void addFloat(String entryName, Supplier<Float> valueSupplier, LogLevel level) {}
    public void addFloat(String entryName, Supplier<Float> valueSupplier) {
        addFloat(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, double value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, double value, LogLevel level) {}

    public void addDouble(String entryName, Supplier<Double> valueSupplier, LogLevel level) {}
    public void addDouble(String entryName, Supplier<Double> valueSupplier) {
        addDouble(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, String value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, String value, LogLevel level) {}

    public void addString(String entryName, Supplier<String> valueSupplier, LogLevel level) {}
    public void addString(String entryName, Supplier<String> valueSupplier) {
        addString(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, byte[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, byte[] value, LogLevel level) {}

    public void addRaw(String entryName, Supplier<byte[]> valueSupplier, LogLevel level) {}
    public void addRaw(String entryName, Supplier<byte[]> valueSupplier) {
        addRaw(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, boolean[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, boolean[] value, LogLevel level) {}

    public void addBooleanArray(String entryName, Supplier<boolean[]> valueSupplier, LogLevel level) {}
    public void addBooleanArray(String entryName, Supplier<boolean[]> valueSupplier) {
        addBooleanArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, int[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, int[] value, LogLevel level) {}

    public void addIntegerArray(String entryName, Supplier<int[]> valueSupplier, LogLevel level) {}
    public void addIntegerArray(String entryName, Supplier<int[]> valueSupplier) {
        addIntegerArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, long[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, long[] value, LogLevel level) {}

    public void addLongArray(String entryName, Supplier<long[]> valueSupplier, LogLevel level) {}
    public void addLongArray(String entryName, Supplier<long[]> valueSupplier) {
        addLongArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, float[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, float[] value, LogLevel level) {}

    public void addFloatArray(String entryName, Supplier<float[]> valueSupplier, LogLevel level) {}
    public void addFloatArray(String entryName, Supplier<float[]> valueSupplier) {
        addFloatArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, double[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, double[] value, LogLevel level) {}

    public void addDoubleArray(String entryName, Supplier<double[]> valueSupplier, LogLevel level) {}
    public void addDoubleArray(String entryName, Supplier<double[]> valueSupplier) {
        addDoubleArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }

    public void put(String entryName, String[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public void put(String entryName, String[] value, LogLevel level) {}

    public void addStringArray(String entryName, Supplier<String[]> valueSupplier, LogLevel level) {}
    public void addStringArray(String entryName, Supplier<String[]> valueSupplier) {
        addStringArray(entryName, valueSupplier, LogLevel.DEFAULT);
    }


    public <R extends StructSerializable> void put(String entryName, R value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public <R extends StructSerializable> void put(String entryName, R value, LogLevel level) {
        putStruct(entryName, (Struct<R>) findStructType(value.getClass()), value, level);
    }

    public <R> void addStruct(String entryName, Struct<R> struct, Supplier<?extends R> valueSupplier, LogLevel level) {}
    public <R> void putStruct(String entryName, Struct<R> struct, R value, LogLevel level) {}
    public <R> void putStruct(String entryName, Struct<R> struct, R value) {
        putStruct(entryName, struct, value, LogLevel.DEFAULT);
    }

    public <R extends StructSerializable> void put(String entryName, R[] value) {
        put(entryName, value, LogLevel.DEFAULT);
    }
    public <R extends StructSerializable> void put(String entryName, R[] value, LogLevel level) {
        Class<R> clazz = (Class<R>) value.getClass().getComponentType();
        putStructArray(entryName, (Struct<R>) findStructType(clazz), value, level);
    }
    public <R> void addStructArray(String entryName, Struct<R> struct, Supplier<R[]> valueSupplier, LogLevel level) {}
    public <R> void putStructArray(String entryName, Struct<R> struct, R[] value, LogLevel level) {}
    public <R> void putStructArray(String entryName, Struct<R> struct, R[] value) {
        putStructArray(entryName, struct, value, LogLevel.DEFAULT);
    }

    public void addSendable(String path, Sendable sendable) {};

    public void addSendable(String pathPrefix, String name, Sendable sendable) {
        String prefix;
        if (!pathPrefix.endsWith("/")) {
            prefix = pathPrefix + "/" + name + "/";
        } else {
            prefix = pathPrefix + name + "/";
        }
        addSendable(prefix, sendable);
    }

    public void addSendable(String pathPrefix, String name, NTSendable sendable) {
        String prefix;
        if (!pathPrefix.endsWith("/")) {
            prefix = pathPrefix + "/" + name + "/";
        } else {
            prefix = pathPrefix + name + "/";
        }
        addSendable(prefix, (Sendable) sendable);
    }

    public void addSendable(String path, NTSendable sendable) {
        addSendable(path, (Sendable) sendable);
    };

    private boolean _isLazy = false;
    public void setLazy(boolean isLazy) {
        this._isLazy = isLazy;
    }
    public boolean isLazy() {return _isLazy;}

    public boolean isNT() {return false;}

    boolean lastDebugValue = true;
    public void update(boolean debug) {
        long timestamp = (long) (Timer.getFPGATimestamp() * 1e6);
        for (Map.Entry<LogLevel, ArrayList<LogRunnable>> levelRunnables : runnableMap.entrySet()) {
            if (lastDebugValue != debug) {
                boolean wasLoggedBefore = levelRunnables.getKey().shouldLog(lastDebugValue, isNT());
                boolean shouldLogNow = levelRunnables.getKey().shouldLog(debug, isNT());
                if (wasLoggedBefore && !shouldLogNow) {
                    levelRunnables.getValue().stream().forEach(
                        (runnable) -> runnable.close()
                    );
                }
                if (!wasLoggedBefore && shouldLogNow) {
                    levelRunnables.getValue().stream().forEach(
                        (runnable) -> runnable.open()
                    );
                }
            }
            if (levelRunnables.getKey().shouldLog(debug, isNT())) {
                levelRunnables.getValue().stream().forEach(
                    (runnable) -> runnable.accept(timestamp)
                );
            }
        }
        sendables.forEach(SendableBuilder::update);
        lastDebugValue = debug;
    }

    protected <T> Struct<T> findStructType(Class<T> classObj) {
        if (StructSerializable.class.isAssignableFrom(classObj)) {
            if (!structTypeCache.containsKey(classObj.getName())) {
                structTypeCache.put(classObj.getName(), null);
                Field field = null;
                try {
                field = classObj.getDeclaredField("struct");
                } catch (NoSuchFieldException | SecurityException e) {
                    DriverStation.reportError("Tried to find struct for type without struct field: " + classObj.toString(), false);
                }

                if (field != null) {
                    if (Struct.class.isAssignableFrom(field.getType())) {
                        try {
                            structTypeCache.put(classObj.getName(), (Struct<T>) field.get(null));
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            DriverStation.reportError("Struct field was not static on: " + classObj.toString(), false);
                        }
                    }
                    else {
                        DriverStation.reportError("Struct field was not a struct on: " + classObj.toString(), false);
                    }
                }
                else {
                    DriverStation.reportError("Struct field was null on: " + classObj.toString(), false);
                }
                
            }
            return (Struct<T>) structTypeCache.get(classObj.getName());
        } else {
            DriverStation.reportError("Tried to find struct type for non-structable type " + classObj.toString(), false);
            return null;
        }
    }

    public static long[] toLongArray(int[] arr) {
        if (arr == null) {return null;}
        long[] newArr = new long[arr.length];
        for (int i = 0; i < arr.length; i++) {
            newArr[i] = (long) arr[i];
        }
        return newArr;
    }

    public <T> void addGeneric(String key, Class<?> type, Supplier<T> supplier, LogLevel level, boolean once) {
        
        Struct<T> struct = (Struct<T>) findStructType(type);
        if (struct != null) {
            if (once){
                putStruct(key, struct, supplier.get(), level);
            } else {
                addStruct(key, struct, supplier, level);
            }
    }}

    public <T> void addGenericArray(String key, Class<?> type, Supplier<?> supplier, LogLevel level, boolean once) {
        if (type.isArray()) {
            Class<T> componentType = (Class<T>) type.getComponentType();
            Struct<T> struct = (Struct<T>) findStructType(componentType);
            if (struct != null) {
                if (once){
                    putStructArray(key, struct, (T[]) supplier.get(), level);
                } else {
                    addStructArray(key, struct, ()->(T[]) supplier.get(), level);
                }
            }
        }

    }

    public <T> void addSupplier(String key, Class<?> type, Supplier<?> supplier, LogLevel level, boolean once) {
    if (type.isArray()) {
      // Array types
      Class<?> componentType = type.getComponentType();
      if (componentType.equals(byte.class)) {
        if (once) {
            put(key, (byte[]) supplier.get(), level);
        } else {
            addRaw(key, () -> (byte[]) supplier.get(), level);
        }
      } else if (componentType.equals(boolean.class)) {
        if (once) {
            put(key, (boolean[]) supplier.get(), level);
        } else {
            addBooleanArray(key, () -> (boolean[]) supplier.get(), level);
        }
      } else if (componentType.equals(int.class)) {
        if (once) {
            put(key, (int[]) supplier.get(), level);
        } else {
            addIntegerArray(key, () -> (int[]) supplier.get(), level);
        }
      } else if (componentType.equals(long.class)) {
        if (once) {
            put(key, (long[]) supplier.get(), level);
        } else {
            addLongArray(key, ()->(long[]) supplier.get(), level);
        }
      } else if (componentType.equals(float.class)) {
        if (once) {
            put(key, (float[]) supplier.get(), level);
        } else {
            addFloatArray(key, ()->(float[]) supplier.get(), level);
        }
      } else if (componentType.equals(double.class)) {
        if (once) {
            put(key, (double[]) supplier.get(), level);
        } else {
            addDoubleArray(key, ()->(double[]) supplier.get(), level);
        }
      } else if (componentType.equals(String.class)) {
        if (once) {
            put(key, (String[]) supplier.get(), level);
        } else {
            addStringArray(key, ()->(String[]) supplier.get(), level);
        }
    } else if (StructSerializable.class.isAssignableFrom(componentType)) {
        addGenericArray(key, type, supplier, level, once);
      
      } else {
        DriverStation.reportError("Non-primitive arrays not supported by Monologue", true);
      }
    } else {
      // Single types
      if (type.equals(boolean.class)) {
        if (once) {
            put(key, (boolean) supplier.get(), level);
        } else {
            addBoolean(key, ()->(boolean) supplier.get(), level);
        }
      } else if (type.equals(int.class)) {
        if (once) {
            put(key, (int) supplier.get(), level);
        } else {
            addInteger(key, ()->(int) supplier.get(), level);
        }
      } else if (type.equals(long.class)) {
        if (once) {
            put(key, (long) supplier.get(), level);
        } else {
            addLong(key, ()->(long) supplier.get(), level);
        }
      } else if (type.equals(float.class)) {
        if (once) {
            put(key, (float) supplier.get(), level);
        } else {
            addFloat(key, ()->(float) supplier.get(), level);
        }
      } else if (type.equals(double.class)) {
        if (once) {
            put(key, (double) supplier.get(), level);
        } else {
            addDouble(key, ()->(double) supplier.get(), level);
        }
      } else if (type.equals(String.class)) {
        if (once) {
            put(key, (String) supplier.get(), level);
        } else {
            addString(key, ()->(String) supplier.get(), level);
        }
      } else if (type.isEnum()) {
        if (once) {
            Object value = supplier.get();
            if (value != null) {
                // Cannot cast to enum subclass, log the name directly
                put(key, ((Enum<?>) value).name(), level);
              } else {
                put(key, "null", level);
              }
        } else {
        addString(key, 
        () -> {
            Object value = supplier.get();
            if (value != null) {
              // Cannot cast to enum subclass, log the name directly
              return ((Enum<?>) value).name();
            } else {
                return "null";
            }
          }, level);
        }
      } else if (StructSerializable.class.isAssignableFrom(type)) {
        addGeneric(key, type, supplier, level, once);
      }
    }
  }
}