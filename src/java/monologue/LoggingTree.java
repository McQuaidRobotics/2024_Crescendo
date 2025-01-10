package monologue;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.lang.invoke.VarHandle;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import monologue.MonoEntryLayer.MonologueBooleanEntry;
import monologue.MonoEntryLayer.MonologueDoubleEntry;
import monologue.MonoEntryLayer.MonologueEntry;
import monologue.MonoEntryLayer.MonologueLongEntry;
import monologue.Primatives.BooleanGetter;
import monologue.Primatives.DoubleGetter;
import monologue.Primatives.LongGetter;

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

  public abstract static class LoggingNode {
    private final String path;

    public LoggingNode(String path) {
      this.path = path;
    }

    public String getPath() {
      return path;
    }

    public boolean isImmutable() {
      return false;
    }

    public LoggingNode asImmutable() {
      return new ImmutableNode(this);
    }

    public LoggingNode asNullable() {
      return new NullableNode(this);
    }

    public LoggingNode asTypeGuarded(Class<?> type) {
      return new TypeGuardedNode(this, type);
    }

    public abstract LoggingNode withNewPath(String path);

    public abstract void log(Object obj);
  }

  public abstract static class ComposableNode extends LoggingNode {
    protected final ArrayList<LoggingNode> children = new ArrayList<>();

    public ComposableNode(String path) {
      super(path);
    }

    public void addChild(LoggingNode child) {
      for (LoggingNode node : children) {
        if (node.path.equals(child.path)) {
          RuntimeLog.warn("Duplicate path: " + child.path);
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
    public void addChild(LoggingNode child) {}

    @Override
    public LoggingNode asImmutable() {
      return this;
    }

    @Override
    public LoggingNode asNullable() {
      return this;
    }

    @Override
    public LoggingNode withNewPath(String path) {
      return new NoopLoggingNode(path);
    }

    @Override
    public void log(Object obj) {}
  }

  public static class ValueNode extends LoggingNode {
    private final Function<Object, Object> getter;
    private final MonologueEntry<Object> entry;

    private final Class<? extends Object> type;
    private final Optional<Struct<Object>> struct;

    @SuppressWarnings("unchecked")
    public ValueNode(
        String path, LogSink sink, Function<Object, Object> getter, Class<? extends Object> type) {
      super(path);
      this.getter = getter;
      this.type = type;
      if (StructSerializable.class.isAssignableFrom(type)) {
        this.struct =
            ProceduralStructGenerator.extractClassStructDynamic(type).map(Struct.class::cast);
        this.entry =
            MonologueEntry.create(
                path,
                struct.orElseGet(
                    () -> {
                      RuntimeLog.warn("No struct for " + type);
                      return ProceduralStructGenerator.noopStruct(Object.class);
                    }),
                sink);
      } else {
        this.struct = Optional.empty();
        this.entry = MonologueEntry.create(path, (Class<Object>) type, sink);
      }
    }

    public ValueNode(
        String path,
        LogSink sink,
        Function<Object, Object> getter,
        Class<? extends Object> type,
        Struct<Object> struct) {
      super(path);
      this.getter = getter;
      this.type = type;
      this.struct = Optional.of(struct);
      this.entry = MonologueEntry.create(path, struct, sink);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      if (struct.isPresent()) {
        return new ValueNode(path, entry.sink(), getter, type, struct.get());
      } else {
        return new ValueNode(path, entry.sink(), getter, type);
      }
    }

    @Override
    public void log(Object obj) {
      entry.log(throwIfNull(getter.apply(obj)));
    }
  }

  public static class DoubleValueNode extends LoggingNode {
    private final DoubleGetter getter;
    private final MonologueDoubleEntry entry;

    public DoubleValueNode(String path, LogSink sink, DoubleGetter getter) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.createDouble(path, sink);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      return new DoubleValueNode(path, entry.sink(), getter);
    }

    @Override
    public void log(Object obj) {
      entry.logDouble(getter.get(obj));
    }
  }

  public static class LongValueNode extends LoggingNode {
    private final LongGetter getter;
    private final MonologueLongEntry entry;

    public LongValueNode(String path, LogSink sink, LongGetter getter) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.createLong(path, sink);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      return new LongValueNode(path, entry.sink(), getter);
    }

    @Override
    public void log(Object obj) {
      entry.logLong(getter.get(obj));
    }
  }

  public static class BooleanValueNode extends LoggingNode {
    private final BooleanGetter getter;
    private final MonologueBooleanEntry entry;

    public BooleanValueNode(String path, LogSink sink, BooleanGetter getter) {
      super(path);
      this.getter = getter;
      this.entry = MonologueEntry.createBoolean(path, sink);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      return new BooleanValueNode(path, entry.sink(), getter);
    }

    @Override
    public void log(Object obj) {
      entry.logBoolean(getter.get(obj));
    }
  }

  @SuppressWarnings("unchecked")
  public static class ValueArrayNode extends LoggingNode {
    private final Function<Object, Object[]> getter;
    private final MonologueEntry<Object[]> entry;

    private final Class<? extends Object> type;
    private final Optional<Struct<Object>> struct;

    public ValueArrayNode(
        String path,
        LogSink sink,
        Function<Object, Object[]> getter,
        Class<? extends Object> type) {
      super(path);
      this.getter = getter;
      this.type = type;
      this.struct = Optional.empty();
      this.entry = MonologueEntry.create(path, (Class<Object[]>) type, sink);
    }

    public ValueArrayNode(
        String path,
        LogSink sink,
        Function<Object, Object[]> getter,
        Class<? extends Object> type,
        Struct<Object> struct) {
      super(path);
      this.getter = getter;
      this.type = type;
      this.struct = Optional.of(struct);
      this.entry = MonologueEntry.create(path, struct, (Class<Object[]>) type, sink);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      if (struct.isPresent()) {
        return new ValueArrayNode(path, entry.sink(), getter, type, struct.get());
      } else {
        return new ValueArrayNode(path, entry.sink(), getter, type);
      }
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

    @Override
    public LoggingNode withNewPath(String path) {
      return new ImmutableNode(node.withNewPath(path));
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

    @Override
    public LoggingNode withNewPath(String path) {
      return new NullableNode(node.withNewPath(path));
    }

    public void log(Object obj) {
      try {
        node.log(obj);
      } catch (MonologueTriedToLogNull e) {
        RuntimeLog.warn(err);
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

    @Override
    public LoggingNode withNewPath(String path) {
      return new TypeGuardedNode(node.withNewPath(path), type);
    }

    public void log(Object obj) {
      if (!type.isInstance(obj)) {
        return;
      }
      node.log(obj);
    }
  }

  public static class ObjectNode extends ComposableNode {
    protected final Function<Object, Object> getter;
    private final String err;
    private final HashSet<Class<?>> seenTypes = new HashSet<>();

    public ObjectNode(String path, Function<Object, Object> getter, Class<?> type) {
      super(path);
      this.getter = getter;
      this.err = path + " is null";
      seenTypes.add(type);
    }

    private ObjectNode(String path, Function<Object, Object> getter, HashSet<Class<?>> seenTypes) {
      super(path);
      this.getter = getter;
      this.err = path + " is null";
      this.seenTypes.addAll(seenTypes);
    }

    @Override
    public LoggingNode withNewPath(String path) {
      var n = new ObjectNode(path, getter, seenTypes);
      for (LoggingNode child : children) {
        n.addChild(child.withNewPath(path + child.getPath().substring(getPath().length())));
      }
      return n;
    }

    protected void updateNodeRegistry(Object o) {
      if (o instanceof Logged && !Logged.getNodes((Logged) o).contains(this)) {
        Logged.addNode((Logged) o, this);
      }
    }

    public void log(Object obj) {
      Object o = getter.apply(obj);
      if (o == null) {
        RuntimeLog.warn(err);
        return;
      }
      if (!seenTypes.contains(o.getClass())) {
        // explore the new class
      }
      updateNodeRegistry(o);
      for (LoggingNode child : children) {
        child.log(o);
      }
    }

    public void logDirect(Object obj) {
      updateNodeRegistry(obj);
      for (LoggingNode child : children) {
        child.log(obj);
      }
    }
  }

  public static class ObjectArrayNode extends ComposableNode {
    private final Function<Object, Object> getter;
    private final String err;
    private final ArrayList<ObjectNode> indexNodes = new ArrayList<>();

    public ObjectArrayNode(String path, Function<Object, Object> getter) {
      super(path);
      this.getter = getter;
      this.err = path + " is null";
      indexNodes.add(getOrCreateIndexNode(0));
    }

    private ObjectNode getOrCreateIndexNode(int index) {
      int highestIndex = indexNodes.size() - 1;
      if (index > highestIndex) {
        for (int i = highestIndex + 1; i <= index; i++) {
          var node = new ObjectNode(getPath() + "/" + i, _v -> _v, Object.class);
          node.addAllChildren(
              children.stream()
                  .map(child -> child.withNewPath(node.getPath() + child.getPath()))
                  .toList());
          indexNodes.add(node);
        }
      }
      return indexNodes.get(index);
    }

    @Override
    public void addChild(LoggingNode child) {
      child = child.withNewPath(child.getPath().substring(getPath().length()));
      super.addChild(child);
      for (int i = 0; i < indexNodes.size(); i++) {
        indexNodes.get(i).addChild(child.withNewPath(getPath() + "/" + i + child.getPath()));
      }
    }

    @Override
    public LoggingNode withNewPath(String path) {
      var n = new ObjectArrayNode(path, getter);
      for (LoggingNode child : children) {
        n.addChild(child.withNewPath(path + child.getPath().substring(getPath().length())));
      }
      // preload the index nodes
      n.getOrCreateIndexNode(indexNodes.size() - 1);
      return n;
    }

    public void log(Object obj) {
      Object[] oa = (Object[]) getter.apply(obj);
      if (oa == null) {
        RuntimeLog.warn(err);
        return;
      }
      for (int i = 0; i < oa.length; i++) {
        getOrCreateIndexNode(i).log(oa[i]);
      }
    }
  }

  public static class OptionalNode extends ObjectNode {
    private final LoggingNode isPresentNode;
    private final String err;

    @SuppressWarnings("unchecked")
    public OptionalNode(String path, LogSink sink, Function<Object, Object> getter, Class<?> type) {
      super(path, getter, type);
      isPresentNode =
          new BooleanValueNode(
              path + "/isPresent", sink, _v -> ((Optional<Object>) _v).isPresent());
      this.err = path + " is null";
    }

    // @SuppressWarnings("unchecked")
    // public OptionalNode(ObjectNode node) {
    //   super(node.getPath(), node.getter, node.seenTypes);
    //   isPresentNode = new BooleanValueNode(
    //       node.getPath() + "/isPresent",
    //       LogSink.NT,
    //       _v -> ((Optional<Object>) _v).isPresent());
    //   this.err = node.getPath() + " is null";
    //   for (LoggingNode child : node.children) {
    //     addChild(child);
    //   }
    // }

    @SuppressWarnings("unchecked")
    public void log(Object obj) {
      Optional<Object> oo = (Optional<Object>) getter.apply(obj);
      if (oo == null) {
        RuntimeLog.warn(err);
        isPresentNode.log(Optional.empty());
        return;
      }
      isPresentNode.log(oo);
      if (oo.isPresent()) {
        super.logDirect(oo.get());
      }
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

    @Override
    public LoggingNode withNewPath(String path) {
      return new StaticObjectNode(path, object);
    }

    public void log(Object obj) {
      for (LoggingNode child : children) {
        child.log(object);
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

    @Override
    public LoggingNode withNewPath(String path) {
      return new SingletonNode(path, type, handle);
    }

    public void log(Object obj) {
      Object o = handle.get(type);
      if (o == null) {
        RuntimeLog.warn(getPath() + " is null");
        return;
      }
      for (LoggingNode child : children) {
        child.log(o);
      }
    }
  }
}
