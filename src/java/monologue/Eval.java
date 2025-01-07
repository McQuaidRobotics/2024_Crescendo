package monologue;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import monologue.Annotations.FlattenedLogged;
import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Annotations.MaybeLoggedType;
import monologue.Annotations.OptionalLogged;
import monologue.Annotations.SingletonLogged;
import monologue.LoggingTree.*;
import monologue.Primatives.BooleanVarHandle;
import monologue.Primatives.DoubleVarHandle;
import monologue.Primatives.LongVarHandle;

public class Eval {
  /**
   * Simplifies the user specified annotations on a field/method to a quanery conditiion expressed
   * as an Optional {@link LogSink}.
   *
   * @param element The field/method to simplify
   * @return The simplified condition
   */
  static Optional<LogSink> annoEval(AccessibleObject element) {
    if (element.isAnnotationPresent(Log.class)) {
      return Optional.of(element.getAnnotation(Log.class).sink());
    } else if (element.isAnnotationPresent(Log.Once.class)) {
      return Optional.of(element.getAnnotation(Log.Once.class).sink());
    } else {
      return Optional.empty();
    }
  }

  /**
   * Checks if their are multiple logging annotations on one field/method.
   *
   * @param element The field/method to check
   * @return If there are too many logging annotations
   */
  static boolean overloadedAnno(AccessibleObject element) {
    int count = 0;
    for (var anno : Annotations.ALL_ANNOTATIONS) {
      if (element.isAnnotationPresent(anno)) {
        count++;
      }
    }
    return count > 1;
  }

  /** Checks if the class is a singleton and returns its key if it is. */
  static Optional<String> singletonKey(Class<?> clazz) {
    if (clazz.isAnnotationPresent(SingletonLogged.class)) {
      return Optional.of(clazz.getAnnotation(SingletonLogged.class).key());
    } else {
      return Optional.empty();
    }
  }

  /** A condensed packaged of what describes a singular logged field/method */
  static class LogMetadata {
    public final boolean annotated;
    public final LogSink sink;
    public final boolean once;
    public final String relativePath;

    private LogMetadata(boolean annotated, LogSink sink, boolean once, String path) {
      this.annotated = annotated;
      this.sink = sink;
      this.once = once;
      this.relativePath = "/" + path;
    }

    /**
     * Derives the metadata from an annotated field/method, if there are no logging annotations this
     * returns null.
     *
     * @param element The field/method to derive the metadata from
     * @return The metadata
     */
    static LogMetadata from(AccessibleObject element) {
      String name;
      if (element instanceof java.lang.reflect.Field) {
        name = ((java.lang.reflect.Field) element).getName();
      } else if (element instanceof java.lang.reflect.Method) {
        name = ((java.lang.reflect.Method) element).getName();
      } else {
        throw new IllegalArgumentException("Only fields and methods can be logged");
      }
      if (element.isAnnotationPresent(Log.class)) {
        Log anno = element.getAnnotation(Log.class);
        return new LogMetadata(true, anno.sink(), false, anno.key().isEmpty() ? name : anno.key());
      } else if (element.isAnnotationPresent(Log.Once.class)) {
        Log.Once anno = element.getAnnotation(Log.Once.class);
        return new LogMetadata(true, anno.sink(), true, anno.key().isEmpty() ? name : anno.key());
      } else {
        return new LogMetadata(false, LogSink.OP, false, name);
      }
    }
  }

  static List<Class<?>> getLoggedInHierarchy(Class<?> type, Class<?> stop) {
    ArrayList<Class<?>> result = new ArrayList<Class<?>>();

    Class<?> i = type;
    while (i != stop && Logged.class.isAssignableFrom(i)) {
      result.add(i);
      i = i.getSuperclass();
    }

    return result;
  }

  static List<Field> getAllFields(List<Class<?>> classes) {
    ArrayList<Field> result = new ArrayList<Field>();
    for (Class<?> clazz : classes) {
      Collections.addAll(result, clazz.getDeclaredFields());
    }
    return result;
  }

  static List<Method> getAllMethods(List<Class<?>> classes) {
    ArrayList<Method> result = new ArrayList<Method>();
    for (Class<?> clazz : classes) {
      Collections.addAll(result, clazz.getDeclaredMethods());
    }
    return result;
  }

  static List<Class<?>> getLoggedClasses(Class<?> type) {
    return getLoggedInHierarchy(type, Object.class);
  }

  static boolean isNestedLogged(Field field) {
    final boolean optional = Optional.class.isAssignableFrom(field.getType());
    final boolean optionalAnnotated = field.isAnnotationPresent(OptionalLogged.class);
    final Class<?> type =
        optional && optionalAnnotated
            ? field.getAnnotation(OptionalLogged.class).type()
            : field.getType();
    final boolean fieldTyLogged =
        Logged.class.isAssignableFrom(type)
            || (type.isArray() && Logged.class.isAssignableFrom(type.getComponentType()));
    final boolean maybeLoggedAnnotation = field.isAnnotationPresent(MaybeLoggedType.class);
    final boolean ignoreLoggedAnnotation = field.isAnnotationPresent(IgnoreLogged.class);
    if (fieldTyLogged && maybeLoggedAnnotation) {
      RuntimeLog.warn(
          field.getName()
              + " of type "
              + field.getType().getSimpleName()
              + " is a Logged type, @MaybeLoggedType is redundant");
    }
    return (fieldTyLogged || maybeLoggedAnnotation) && !ignoreLoggedAnnotation;
  }

  static VarHandle getHandle(Field field, MethodHandles.Lookup lookup) {
    try {
      var privateLookup = MethodHandles.privateLookupIn(field.getDeclaringClass(), lookup);
      return privateLookup.unreflectVarHandle(field);
    } catch (IllegalAccessException e) {
      RuntimeLog.warn(
          "Could not access field "
              + field.getName()
              + " of type "
              + field.getType().getSimpleName()
              + " in "
              + field.getDeclaringClass().getSimpleName()
              + ": "
              + e.getMessage());
      return null;
    }
  }

  static MethodHandle getHandle(Method method, MethodHandles.Lookup lookup) {
    try {
      var privateLookup = MethodHandles.privateLookupIn(method.getDeclaringClass(), lookup);
      return privateLookup.unreflect(method);
    } catch (IllegalAccessException e) {
      RuntimeLog.warn(
          "Could not access field "
              + method.getName()
              + " in "
              + method.getDeclaringClass().getSimpleName()
              + ": "
              + e.getMessage());
      return null;
    }
  }

  static <LN extends ComposableNode> LN exploreNodes(List<Class<?>> types, final LN rootNode) {
    final List<Field> fields = getAllFields(types);
    final List<Method> methods = getAllMethods(types);
    final MethodHandles.Lookup lookup = MethodHandles.lookup();
    final String rootPath = rootNode.getPath();

    for (final Field field : fields) {
      final boolean isNestedLogged = isNestedLogged(field);
      final boolean isValidLiteralType = TypeChecker.isValidLiteralType(field.getType());
      final boolean isStatic = Modifier.isStatic(field.getModifiers());
      final boolean isArray = field.getType().isArray();
      final LogMetadata metadata = LogMetadata.from(field);
      final boolean optional = Optional.class.isAssignableFrom(field.getType());
      final boolean optionalAnnotated = field.isAnnotationPresent(OptionalLogged.class);
      if (!isNestedLogged && !isValidLiteralType) {
        continue;
      }
      if ((optional && !isNestedLogged) || (optional && !optionalAnnotated)) {
        continue;
      }
      final VarHandle handle = getHandle(field, lookup);
      if (handle == null) {
        continue;
      }

      // handle singletons
      if (isNestedLogged && isStatic) {
        Optional<String> singletonKey = singletonKey(field.getType());
        if (singletonKey.isPresent() && !Logged.singletonAlreadyAdded(field.getType())) {
          try {
            Monologue.logTree((Logged) field.get(null), singletonKey.get());
            Logged.addSingleton(
                field.getType(), new SingletonNode(singletonKey.get(), field.getType(), handle));
          } catch (IllegalAccessException e) {
            RuntimeLog.warn("Issue with singleton " + field.getType().getSimpleName());
          }
        }
        continue;
      } else if (metadata.annotated && isStatic) {
        RuntimeLog.warn(
            "Static field "
                + field.getName()
                + " of type "
                + field.getType().getSimpleName()
                + " in "
                + rootPath
                + " is will not be logged");
        continue;
      }

      final Class<?> type;
      if (optional && optionalAnnotated) {
        type = field.getAnnotation(OptionalLogged.class).type();
      } else {
        type = field.getType();
      }

      if (isArray && isNestedLogged) {
        ComposableNode node = new ObjectArrayNode(rootPath + metadata.relativePath, handle::get);
        rootNode.addChild(exploreNodes(getLoggedClasses(type.getComponentType()), node));
      } else if (isNestedLogged) {
        boolean isFlattened = field.isAnnotationPresent(FlattenedLogged.class);
        String relativePath = isFlattened ? "" : metadata.relativePath;
        final ObjectNode node;
        if (optional) {
          node = new OptionalNode(rootPath + relativePath, metadata.sink, handle::get, type);
        } else {
          node = new ObjectNode(rootPath + relativePath, handle::get, type);
        }
        rootNode.addChild(exploreNodes(getLoggedClasses(type), node));
      } else if (isValidLiteralType) {
        if (!metadata.annotated || overloadedAnno(field)) {
          continue;
        }
        final boolean isFinal = Modifier.isFinal(field.getModifiers());
        final boolean isPrimitive = type.isPrimitive();
        LoggingNode node;
        if (type.isArray()) {
          node =
              new ValueArrayNode(
                  rootPath + metadata.relativePath,
                  metadata.sink,
                  obj -> (Object[]) handle.get(obj),
                  type);
        } else if (type == double.class
            || type == Double.TYPE
            || type == float.class
            || type == Float.TYPE) {
          node =
              new DoubleValueNode(
                  rootPath + metadata.relativePath,
                  metadata.sink,
                  new DoubleVarHandle(handle)::get);
        } else if ((type == long.class
            || type == Long.TYPE
            || type == int.class
            || type == Integer.TYPE)) {
          node =
              new LongValueNode(
                  rootPath + metadata.relativePath, metadata.sink, new LongVarHandle(handle)::get);
        } else if (type == boolean.class || type == Boolean.TYPE) {
          node =
              new BooleanValueNode(
                  rootPath + metadata.relativePath,
                  metadata.sink,
                  new BooleanVarHandle(handle)::get);
        } else {
          node =
              new ValueNode(
                  rootPath + metadata.relativePath, metadata.sink, handle::get, field.getType());
        }
        if (isFinal && isPrimitive) {
          node = node.asImmutable();
        } else if (!isPrimitive) {
          node = node.asNullable();
        }
        rootNode.addChild(node);
      }
    }

    for (final Method method : methods) {
      LogMetadata metadata = LogMetadata.from(method);
      if (!metadata.annotated
          || method.getParameterCount() > 0
          || !TypeChecker.isValidLiteralType(method.getReturnType())) {
        continue;
      }
      MethodHandle handle = getHandle(method, lookup);
      if (handle == null) {
        continue;
      }

      final String err = "Could not invoke method " + method.getName() + " in " + rootPath + ": ";

      boolean isPrimitive = method.getReturnType().isPrimitive();
      LoggingNode node;
      if (method.getReturnType().isArray()) {
        node =
            new ValueArrayNode(
                metadata.relativePath,
                metadata.sink,
                obj -> {
                  try {
                    return (Object[]) handle.invoke(obj);
                  } catch (Throwable e) {
                    RuntimeLog.warn(err + e.getMessage());
                    return null;
                  }
                },
                method.getReturnType());
      } else {
        node =
            new ValueNode(
                rootPath + metadata.relativePath,
                metadata.sink,
                obj -> {
                  try {
                    return handle.invoke(obj);
                  } catch (Throwable e) {
                    RuntimeLog.warn(err + e.getMessage());
                    return null;
                  }
                },
                method.getReturnType());
      }
      if (!isPrimitive) {
        node = node.asNullable();
      }
      rootNode.addChild(node);
    }

    return rootNode;
  }
}
