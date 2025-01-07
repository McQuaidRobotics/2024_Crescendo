package monologue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import monologue.MonoEntryLayer.MonologueBooleanEntry;
import monologue.MonoEntryLayer.MonologueDoubleEntry;
import monologue.MonoEntryLayer.MonologueEntry;
import monologue.MonoEntryLayer.MonologueLongEntry;

public class MonoSendableLayer {
  private static final ArrayList<SendableContainer> sendables = new ArrayList<>();

  static void updateAll() {
    for (SendableContainer sendable : sendables) {
      sendable.update();
    }
  }

  static void addSendableContainer(SendableContainer container) {
    container.postConstants();
    sendables.add(container);
  }

  static class SendableContainer {
    final ArrayList<Runnable> updates = new ArrayList<>();
    final ArrayList<Runnable> constants = new ArrayList<>();
    final LogSink sink;

    SendableContainer(LogSink sink) {
      this.sink = sink;
    }

    void addUpdatable(Runnable r) {
      updates.add(r);
    }

    void addConstant(Runnable r) {
      constants.add(r);
    }

    void update() {
      updates.forEach(Runnable::run);
    }

    void postConstants() {
      constants.forEach(Runnable::run);
    }
  }

  static class Builder implements SendableBuilder {
    private final String path;
    private final SendableContainer sendable;
    private final LogSink sink;

    Builder(String path, LogSink sink) {
      this.path = path;
      this.sendable = new SendableContainer(sink);
      this.sink = sink;
    }

    void finish() {
      addSendableContainer(sendable);
    }

    @Override
    public void setSmartDashboardType(String type) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<String> entry =
                MonologueEntry.create(path + "/.type", String.class, sink);

            public void run() {
              entry.log(type);
            }
            ;
          });
    }

    @Override
    public void setActuator(boolean value) {}

    @Override
    public void setSafeState(Runnable func) {}

    @Override
    public void addCloseable(AutoCloseable closeable) {}

    @Override
    public void clearProperties() {}

    @Override
    public BackendKind getBackendKind() {
      return BackendKind.kUnknown;
    }

    @Override
    public boolean isPublished() {
      return true;
    }

    @Override
    public void close() throws Exception {}

    @Override
    public void update() {}

    @Override
    public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueBooleanEntry entry = MonologueEntry.createBoolean(path + "/" + key, sink);

            public void run() {
              entry.logBoolean(getter.getAsBoolean());
            }
            ;
          });
    }

    @Override
    public void addBooleanArrayProperty(
        String key, Supplier<boolean[]> getter, Consumer<boolean[]> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<boolean[]> entry =
                MonologueEntry.create(path + "/" + key, boolean[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstBoolean(String key, boolean value) {
      sendable.addConstant(
          new Runnable() {
            MonologueBooleanEntry entry = MonologueEntry.createBoolean(path + "/" + key, sink);

            public void run() {
              entry.logBoolean(value);
            }
            ;
          });
    }

    @Override
    public void publishConstBooleanArray(String key, boolean[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<boolean[]> entry =
                MonologueEntry.create(path + "/" + key, boolean[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void addIntegerProperty(String key, LongSupplier getter, LongConsumer _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueLongEntry entry = MonologueEntry.createLong(path + "/" + key, sink);

            public void run() {
              entry.logLong(getter.getAsLong());
            }
            ;
          });
    }

    @Override
    public void addIntegerArrayProperty(
        String key, Supplier<long[]> getter, Consumer<long[]> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<long[]> entry =
                MonologueEntry.create(path + "/" + key, long[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstInteger(String key, long value) {
      sendable.addConstant(
          new Runnable() {
            MonologueLongEntry entry = MonologueEntry.createLong(path + "/" + key, sink);

            public void run() {
              entry.logLong(value);
            }
            ;
          });
    }

    @Override
    public void publishConstIntegerArray(String key, long[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<long[]> entry =
                MonologueEntry.create(path + "/" + key, long[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueDoubleEntry entry = MonologueEntry.createDouble(path + "/" + key, sink);

            public void run() {
              entry.logDouble(getter.getAsDouble());
            }
            ;
          });
    }

    @Override
    public void addDoubleArrayProperty(
        String key, Supplier<double[]> getter, Consumer<double[]> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<double[]> entry =
                MonologueEntry.create(path + "/" + key, double[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstDouble(String key, double value) {
      sendable.addConstant(
          new Runnable() {
            MonologueDoubleEntry entry = MonologueEntry.createDouble(path + "/" + key, sink);

            public void run() {
              entry.logDouble(value);
            }
            ;
          });
    }

    @Override
    public void publishConstDoubleArray(String key, double[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<double[]> entry =
                MonologueEntry.create(path + "/" + key, double[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueDoubleEntry entry = MonologueEntry.createDouble(path + "/" + key, sink);

            public void run() {
              entry.logDouble(getter.getAsFloat());
            }
            ;
          });
    }

    @Override
    public void addFloatArrayProperty(
        String key, Supplier<float[]> getter, Consumer<float[]> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<float[]> entry =
                MonologueEntry.create(path + "/" + key, float[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstFloat(String key, float value) {
      sendable.addConstant(
          new Runnable() {
            MonologueDoubleEntry entry = MonologueEntry.createDouble(path + "/" + key, sink);

            public void run() {
              entry.logDouble(value);
            }
            ;
          });
    }

    @Override
    public void publishConstFloatArray(String key, float[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<float[]> entry =
                MonologueEntry.create(path + "/" + key, float[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<String> entry =
                MonologueEntry.create(path + "/" + key, String.class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void addStringArrayProperty(
        String key, Supplier<String[]> getter, Consumer<String[]> _setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<String[]> entry =
                MonologueEntry.create(path + "/" + key, String[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstString(String key, String value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<String> entry =
                MonologueEntry.create(path + "/" + key, String.class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void publishConstStringArray(String key, String[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<String[]> entry =
                MonologueEntry.create(path + "/" + key, String[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }

    @Override
    public void addRawProperty(
        String key, String typeString, Supplier<byte[]> getter, Consumer<byte[]> setter) {
      if (getter == null) return;
      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<byte[]> entry =
                MonologueEntry.create(path + "/" + key, byte[].class, sink);

            public void run() {
              entry.log(getter.get());
            }
            ;
          });
    }

    @Override
    public void publishConstRaw(String key, String typeString, byte[] value) {
      sendable.addConstant(
          new Runnable() {
            MonologueEntry<byte[]> entry =
                MonologueEntry.create(path + "/" + key, byte[].class, sink);

            public void run() {
              entry.log(value);
            }
            ;
          });
    }
  }

  static class NtSendableCompat {
    static final VarHandle field2dObject;
    static final VarHandle field2dObjectName;
    static final VarHandle field2dObjectPoses;

    static final VarHandle mechanism2dDims;
    static final VarHandle mechanism2dColor;
    static final VarHandle mechanism2dRoots;
    static final VarHandle mechanism2dRootX;
    static final VarHandle mechanism2dRootY;
    static final VarHandle mechanism2dLigamentAngle;
    static final VarHandle mechanism2dLigamentColor;
    static final VarHandle mechanism2dLigamentLength;
    static final VarHandle mechanism2dLigamentWeight;
    static final VarHandle mechanism2dObjects;

    static {
      final MethodHandles.Lookup lookup = MethodHandles.lookup();
      try {
        MethodHandles.Lookup field2dLookup = MethodHandles.privateLookupIn(Field2d.class, lookup);
        MethodHandles.Lookup fieldObjectLookup =
            MethodHandles.privateLookupIn(FieldObject2d.class, lookup);
        field2dObject = field2dLookup.findVarHandle(Field2d.class, "m_objects", List.class);
        field2dObjectName =
            fieldObjectLookup.findVarHandle(FieldObject2d.class, "m_name", String.class);
        field2dObjectPoses =
            fieldObjectLookup.findVarHandle(FieldObject2d.class, "m_poses", List.class);

        MethodHandles.Lookup mechanism2dLookup =
            MethodHandles.privateLookupIn(Mechanism2d.class, lookup);
        MethodHandles.Lookup rootLookup =
            MethodHandles.privateLookupIn(MechanismRoot2d.class, lookup);
        MethodHandles.Lookup ligamentLookup =
            MethodHandles.privateLookupIn(MechanismLigament2d.class, lookup);
        MethodHandles.Lookup objectLookup =
            MethodHandles.privateLookupIn(MechanismObject2d.class, lookup);
        mechanism2dDims =
            mechanism2dLookup.findVarHandle(Mechanism2d.class, "m_dims", double[].class);
        mechanism2dColor =
            mechanism2dLookup.findVarHandle(Mechanism2d.class, "m_color", String.class);
        mechanism2dRoots = mechanism2dLookup.findVarHandle(Mechanism2d.class, "m_roots", Map.class);
        mechanism2dRootX = rootLookup.findVarHandle(MechanismRoot2d.class, "m_x", double.class);
        mechanism2dRootY = rootLookup.findVarHandle(MechanismRoot2d.class, "m_y", double.class);
        mechanism2dLigamentAngle =
            ligamentLookup.findVarHandle(MechanismLigament2d.class, "m_angle", double.class);
        mechanism2dLigamentColor =
            ligamentLookup.findVarHandle(MechanismLigament2d.class, "m_color", String.class);
        mechanism2dLigamentLength =
            ligamentLookup.findVarHandle(MechanismLigament2d.class, "m_length", double.class);
        mechanism2dLigamentWeight =
            ligamentLookup.findVarHandle(MechanismLigament2d.class, "m_weight", double.class);
        mechanism2dObjects =
            objectLookup.findVarHandle(MechanismObject2d.class, "m_objects", Map.class);
      } catch (NoSuchFieldException | IllegalAccessException e) {
        throw new RuntimeException(e);
      }
    }

    public static void addField2d(String path, Field2d field, LogSink sink) {
      SendableContainer sendable = new SendableContainer(sink);
      List<FieldObject2d> objects = (List<FieldObject2d>) field2dObject.get(field);

      sendable.addUpdatable(
          new Runnable() {
            public void run() {
              for (FieldObject2d object : objects) {
                String name = (String) field2dObjectName.get(object);
                MonologueEntry<double[]> entry =
                    MonologueEntry.create(path + "/" + name, double[].class, sink);
                List<Pose2d> poses = (List<Pose2d>) field2dObjectPoses.get(object);
                double[] arr = new double[3 * poses.size()];
                int ndx = 0;
                for (Pose2d pose : poses) {
                  var translation = pose.getTranslation();
                  arr[ndx + 0] = translation.getX();
                  arr[ndx + 1] = translation.getY();
                  arr[ndx + 2] = pose.getRotation().getDegrees();
                  ndx += 3;
                }
                entry.log(arr);
              }
            }
            ;
          });

      sendable.addConstant(
          new Runnable() {
            MonologueEntry<String> entry =
                MonologueEntry.create(path + "/.type", String.class, sink);

            public void run() {
              entry.log("Field2d");
            }
            ;
          });

      addSendableContainer(sendable);
    }

    public static void addMechanism2dLigament(
        String path, MechanismLigament2d ligament, SendableContainer sendable, LogSink sink) {
      double angle = (double) mechanism2dLigamentAngle.get(ligament);
      String color = (String) mechanism2dLigamentColor.get(ligament);
      double length = (double) mechanism2dLigamentLength.get(ligament);
      double weight = (double) mechanism2dLigamentWeight.get(ligament);

      sendable.addUpdatable(
          new Runnable() {
            MonologueDoubleEntry angleEntry = MonologueEntry.createDouble(path + "/angle", sink);
            MonologueEntry<String> colorEntry =
                MonologueEntry.create(path + "/color", String.class, sink);
            MonologueDoubleEntry lengthEntry = MonologueEntry.createDouble(path + "/length", sink);
            MonologueDoubleEntry weightEntry = MonologueEntry.createDouble(path + "/weight", sink);

            public void run() {
              angleEntry.logDouble(angle);
              colorEntry.log(color);
              lengthEntry.logDouble(length);
              weightEntry.logDouble(weight);
            }
            ;
          });
    }

    public static void addMechanism2dRoot(
        String path, MechanismRoot2d root, SendableContainer sendable, LogSink sink) {
      double x = (double) mechanism2dRootX.get(root);
      double y = (double) mechanism2dRootY.get(root);

      sendable.addUpdatable(
          new Runnable() {
            MonologueDoubleEntry xEntry = MonologueEntry.createDouble(path + "/x", sink);
            MonologueDoubleEntry yEntry = MonologueEntry.createDouble(path + "/y", sink);

            public void run() {
              xEntry.logDouble(x);
              yEntry.logDouble(y);
            }
            ;
          });
    }

    public static void addMechanism2dObject(
        String path, MechanismObject2d object, SendableContainer sendable, LogSink sink) {
      Map<String, MechanismObject2d> objects =
          (Map<String, MechanismObject2d>) mechanism2dObjects.get(object);

      if (object instanceof MechanismLigament2d) {
        addMechanism2dLigament(path, (MechanismLigament2d) object, sendable, sink);
      } else if (object instanceof MechanismRoot2d) {
        addMechanism2dRoot(path, (MechanismRoot2d) object, sendable, sink);
      }

      sendable.addUpdatable(
          new Runnable() {
            public void run() {
              for (Map.Entry<String, MechanismObject2d> entry : objects.entrySet()) {
                addMechanism2dObject(path + "/" + entry.getKey(), entry.getValue(), sendable, sink);
              }
            }
            ;
          });
    }

    public static void addMechanism2d(String path, Mechanism2d mech, LogSink sink) {
      SendableContainer sendable = new SendableContainer(sink);
      Map<String, MechanismRoot2d> roots =
          (Map<String, MechanismRoot2d>) mechanism2dRoots.get(mech);

      sendable.addUpdatable(
          new Runnable() {
            MonologueEntry<double[]> dimsEntry =
                MonologueEntry.create(path + "/dims", double[].class, sink);
            MonologueEntry<String> colorEntry =
                MonologueEntry.create(path + "/color", String.class, sink);

            public void run() {
              dimsEntry.log((double[]) mechanism2dDims.get(mech));
              colorEntry.log((String) mechanism2dColor.get(mech));
            }
            ;
          });

      for (Map.Entry<String, MechanismRoot2d> entry : roots.entrySet()) {
        addMechanism2dObject(path + "/" + entry.getKey(), entry.getValue(), sendable, sink);
      }

      sendable.addConstant(
          new Runnable() {
            MonologueEntry<String> entry =
                MonologueEntry.create(path + "/.type", String.class, sink);

            public void run() {
              entry.log("Mechanism2d");
            }
            ;
          });

      addSendableContainer(sendable);
    }
  }
}
