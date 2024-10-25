package monologue;

import java.lang.invoke.VarHandle;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.MonologueEntryLayer.MonologueEntry;

public class LoggingTree {
  private static class MonologueTriedToLogNull extends NullPointerException {
    private static final long serialVersionUID = 1L;
  }

  private static <T> T throwIfNull(T obj) {
    if (obj == null) {
      throw new MonologueTriedToLogNull();
    }
    return obj;
  }

  public static abstract class LoggingNode {
    private final String path;

    public LoggingNode(String path) {
      this.path = path;
    }

    public String getPath() {
      return path;
    }

    public LoggingNode asImmutable() {
      return new ImmutableNode(this);
    }

    public boolean isImmutable() {
      return false;
    }

    public LoggingNode asNullable() {
      return new NullableNode(this);
    }

    public LoggingNode asTypeGuarded(Class<?> type) {
      return new TypeGuardedNode(this, type);
    }

    public abstract void log(Object obj);
  }

  public static abstract class ComposableNode extends LoggingNode {
    protected final ArrayList<LoggingNode> children = new ArrayList<>();

    public ComposableNode(String path) {
      super(path);
    }

    public void addChild(LoggingNode child) {
      for (LoggingNode node : children) {
        if (node.path.equals(child.path)) {
          MonologueLog.runtimeWarn("Duplicate path: " + child.path);
          return;
        }
      }
      children.add(child);
    }

    public void addAllChildren(List<LoggingNode> children) {
      for (LoggingNode child : children) {
        addChild(child);
      }
    }
  }

  public static class NoopLoggingNode extends ComposableNode {
    public NoopLoggingNode(String path) {
      super(path);
    }

    @Override
    public void addChild(LoggingNode child) {
    }

    @Override
    public LoggingNode asImmutable() {
      return this;
    }

    @Override
    public LoggingNode asNullable() {
      return this;
    }

    public void log(Object obj) {
    }
  }

  public static class ValueNode extends LoggingNode {
    private final Function<Object, Object> getter;
    private final MonologueEntry<Object> entry;

    @SuppressWarnings("unchecked")
    public ValueNode(String path, LogSink sink, Function<Object, Object> getter, Class<? extends Object> type) {
      super(path);
      this.getter = getter;
      if (StructSerializable.class.isAssignableFrom(type)) {
        this.entry = MonologueEntry.create(
          path,
          (Struct<Object>) ProceduralStructGenerator.extractClassStructDynamic(type).get(),
          (Class<Object>) type,
          sink
        );
      } else {
        this.entry = MonologueEntry.create(path, (Class<Object>) type, sink);
      }
    }

    @SuppressWarnings("unchecked")
    public ValueNode(String path, LogSink sink, Function<Object, Object> getter, Class<? extends Object> type,
        Struct<Object> struct) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.create(path, struct, (Class<Object>) type, sink);
    }

    public void log(Object obj) {
      entry.log(throwIfNull(getter.apply(obj)));
    }
  }

  @SuppressWarnings("unchecked")
  public static class ValueArrayNode extends LoggingNode {
    private final Function<Object, Object[]> getter;
    private final MonologueEntry<Object[]> entry;

    public ValueArrayNode(String path, LogSink sink, Function<Object, Object[]> getter, Class<? extends Object> type) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.create(path, (Class<Object[]>) type, sink);
    }

    public ValueArrayNode(String path, LogSink sink, Function<Object, Object[]> getter, Class<? extends Object> type,
        Struct<Object> struct) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.createStructArray(path, struct, (Class<Object[]>) type, sink);
    }

    public void log(Object obj) {
      entry.log(throwIfNull(getter.apply(obj)));
    }
  }

  public static class ImmutableNode extends LoggingNode {
    private final LoggingNode node;
    private boolean logged = false;

    public ImmutableNode(LoggingNode node) {
      super(node.path);
      this.node = node;
    }

    @Override
    public boolean isImmutable() {
      return true;
    }

    public void log(Object obj) {
      if (!logged) {
        node.log(obj);
        logged = true;
      }
    }
  }

  public static class NullableNode extends LoggingNode {
    private final LoggingNode node;
    private final String err;

    public NullableNode(LoggingNode node) {
      super(node.path);
      this.node = node;
      this.err = node.path + " is null";
    }

    public void log(Object obj) {
      try {
        node.log(obj);
      } catch (MonologueTriedToLogNull e) {
        MonologueLog.runtimeWarn(err);
      }
    }
  }

  public static class TypeGuardedNode extends LoggingNode {
    private final LoggingNode node;
    private final Class<?> type;

    public TypeGuardedNode(LoggingNode node, Class<?> type) {
      super(node.path);
      this.node = node;
      this.type = type;
    }

    public void log(Object obj) {
      if (!type.isInstance(obj)) {
        return;
      }
      node.log(obj);
    }
  }

  public static class ObjectNode extends ComposableNode {
    protected final VarHandle handle;
    private final String err;
    private final ArrayList<Class<?>> seenTypes = new ArrayList<>();

    public ObjectNode(String path, VarHandle handle) {
      super(path);
      this.handle = handle;
      this.err = path + " is null";
      seenTypes.add(handle.varType());
    }

    public void log(Object obj) {
      Object o = handle.get(obj);
      if (o == null) {
        MonologueLog.runtimeWarn(err);
        return;
      }
      if (!seenTypes.contains(o.getClass())) {
        // explore the new class
      }
      if (o instanceof Logged && Logged.getNodes((Logged) o).isEmpty()) {
        Logged.addNode((Logged) o, this);
      }
      for (LoggingNode child : children) {
        child.log(o);
      }
    }
  }

  public static class OptionalNode extends ObjectNode {
    private final LoggingNode isPresentNode;
    private final String err;

    @SuppressWarnings("unchecked")
    public OptionalNode(String path, LogSink sink, VarHandle handle) {
      super(path, handle);
      isPresentNode = new ValueNode(
          path + "/isPresent",
          sink,
          _v -> ((Optional<Object>) _v).isPresent(),
          Boolean.class);
      this.err = path + " is null";
    }

    public void log(Object obj) {
      Optional<Object> oo = (Optional<Object>) handle.get(obj);
      if (oo == null) {
        MonologueLog.runtimeWarn(err);
        isPresentNode.log(Optional.empty());
        return;
      }
      isPresentNode.log(oo);
      oo.ifPresent(super::log);
    }
  }

  public static class StaticObjectNode extends ComposableNode {
    private final Object object;

    public StaticObjectNode(String path, Object object) {
      super(path);
      this.object = object;
    }

    @Override
    public boolean isImmutable() {
      return true;
    }

    public void log(Object obj) {
      for (LoggingNode child : children) {
        child.log(object);
      }
    }
  }

  public static class ObjectArrayNode extends ComposableNode {
    private final VarHandle handle;
    private final String err;

    public ObjectArrayNode(String path, VarHandle handle) {
      super(path);
      this.handle = handle;
      this.err = path + " is null";
    }

    public void log(Object obj) {
      Object[] o = (Object[]) handle.get(obj);
      if (o == null) {
        MonologueLog.runtimeWarn(err);
        return;
      }
      for (LoggingNode child : children) {
        for (int i = 0; i < o.length; i++) {
          child.log(o[i]);
        }
      }
    }
  }

  public static class SingletonNode extends ComposableNode {
    private final Class<?> type;
    private final VarHandle handle;

    public SingletonNode(String path, Class<?> type, VarHandle handle) {
      super(path);
      this.type = type;
      this.handle = handle;
    }

    public void log(Object obj) {
      Object o = handle.get(type);
      if (o == null) {
        MonologueLog.runtimeWarn(getPath() + " is null");
        return;
      }
      for (LoggingNode child : children) {
        child.log(o);
      }
    }
  }
}
