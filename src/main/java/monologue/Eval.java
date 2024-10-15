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

import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Annotations.MaybeLoggedType;
import monologue.Annotations.SingletonLogged;
import monologue.LoggingTree.*;

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

  /**
   * Checks if the class is a singleton and returns its key if it is.
   */
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
        return new LogMetadata(false, null, false, "");
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
    boolean fieldTyLogged = Logged.class.isAssignableFrom(field.getType());
    boolean maybeLoggedAnnotation = field.isAnnotationPresent(MaybeLoggedType.class);
    boolean ignoreLoggedAnnotation = field.isAnnotationPresent(IgnoreLogged.class);
    if (fieldTyLogged && maybeLoggedAnnotation) {
      MonologueLog.runtimeWarn(
          field.getName()
              + " of type "
              + field.getType().getSimpleName()
              + " is a Logged type, @MaybeLoggedType is redundant");
    }
    return (fieldTyLogged || maybeLoggedAnnotation) && !ignoreLoggedAnnotation;
  }

  static <LN extends ComposableNode> LN exploreNodes(List<Class<?>> types, LN rootNode) {
    final List<Field> fields = getAllFields(types);
    final List<Method> methods = getAllMethods(types);
    final MethodHandles.Lookup lookup = MethodHandles.lookup();
    final String rootPath = rootNode.getPath();

    for (Field field : fields) {
      boolean isNestedLogged = isNestedLogged(field);
      boolean isValidLiteralType = TypeChecker.isValidLiteralType(field.getType());
      if (!isNestedLogged && !isValidLiteralType) {
        continue;
      }
      VarHandle handle;
      try {
        var privateLookup = MethodHandles.privateLookupIn(field.getDeclaringClass(), lookup);
        handle = privateLookup.unreflectVarHandle(field);
      } catch (IllegalAccessException e) {
        MonologueLog.runtimeWarn(
            "Could not access field "
                + field.getName()
                + " of type "
                + field.getType().getSimpleName()
                + " in "
                + rootPath
                + ": "
                + e.getMessage());
        continue;
      }
      if (isNestedLogged) {
        ComposableNode node = new ObjectNode(
          rootPath + "/" + field.getName(),
          handle
        );
        rootNode.addChild(exploreNodes(getLoggedClasses(field.getType()), node));
      } else if (isValidLiteralType) {
        LogMetadata metadata = LogMetadata.from(field);
        if (!metadata.annotated || overloadedAnno(field)) {
          continue;
        }
        boolean isFinal = Modifier.isFinal(field.getModifiers());
        boolean isPrimitive = field.getType().isPrimitive();
        LoggingNode node;
        if (field.getType().isArray()) {
          node = new ValueArrayNode(
              rootPath + metadata.relativePath,
              metadata.sink,
              obj -> (Object[]) handle.get(obj),
              field.getType());
        } else {
          node = new ValueNode(
              rootPath + metadata.relativePath,
              metadata.sink,
              handle::get,
              field.getType());
        }
        if (isFinal && isPrimitive) {
          node = node.asImmutable();
        } else if (!isPrimitive) {
          node = node.asNullable();
        }
        rootNode.addChild(node);
      }
    }

    for (Method method : methods) {
      LogMetadata metadata = LogMetadata.from(method);
      if (!metadata.annotated || method.getParameterCount() > 0
          || !TypeChecker.isValidLiteralType(method.getReturnType())) {
        continue;
      }
      method.setAccessible(true);
      MethodHandle handle;
      try {
        handle = lookup.unreflect(method);
      } catch (IllegalAccessException e) {
        MonologueLog.runtimeWarn(
            "Could not access method "
                + method.getName()
                + " in "
                + rootPath
                + ": "
                + e.getMessage());
        continue;
      }
      boolean isPrimitive = method.getReturnType().isPrimitive();
      LoggingNode node;
      if (method.getReturnType().isArray()) {
        node = new ValueArrayNode(
            metadata.relativePath,
            metadata.sink,
            obj -> {
              try { return (Object[]) handle.invoke(obj); }
              catch (Throwable e) {
                MonologueLog.runtimeWarn(
                    "Could not invoke method "
                        + method.getName()
                        + " in "
                        + rootPath
                        + ": "
                        + e.getMessage());
                return null;
              }
            },
            method.getReturnType());
      } else {
        node = new ValueNode(
            rootPath + metadata.relativePath,
            metadata.sink,
            obj -> {
              try { return handle.invoke(obj); }
              catch (Throwable e) {
                MonologueLog.runtimeWarn(
                    "Could not invoke method "
                        + method.getName()
                        + " in "
                        + rootPath
                        + ": "
                        + e.getMessage());
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
