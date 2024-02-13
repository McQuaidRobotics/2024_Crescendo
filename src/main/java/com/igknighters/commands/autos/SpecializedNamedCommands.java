package com.igknighters.commands.autos;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class SpecializedNamedCommands {
    private final static HashMap<String, SpecializedNamedCommand> commands = new HashMap<>();

    private static Object fromString(String s, Class<?> clazz) {
        if (clazz == Integer.class) {
            return clazz.cast(Integer.parseInt(s));
        } else if (clazz == Double.class) {
            return clazz.cast(Double.parseDouble(s));
        } else if (clazz == Boolean.class) {
            return clazz.cast(Boolean.parseBoolean(s));
        } else {
            throw new IllegalArgumentException("Unsupported class type");
        }
    }

    public record SpecializedNamedCommand(Function<Object[], Command> fn, Class<?>... params) {
        Command construct(String... args) {
            if (params.length != args.length) {
                throw new IllegalArgumentException("Incorrect number of parameters");
            }
            Object[] paramObjects = new Object[params.length];
            for (int i = 0; i < params.length; i++) {
                paramObjects[i] = params[i].cast(fromString(args[i], params[i]));
            }
            return fn.apply(paramObjects);
        }

        Command construct(Object... args) {
            if (params.length != args.length) {
                throw new IllegalArgumentException("Incorrect number of parameters");
            }
            Object[] paramObjects = new Object[params.length];
            for (int i = 0; i < params.length; i++) {
                paramObjects[i] = params[i].cast(args[i]);
            }
            return fn.apply(paramObjects);
        }

        public static SpecializedNamedCommand fromMethod(Class<?> clazz, String methodName, Class<?>... params) {
            return new SpecializedNamedCommand(args -> {
                try {
                    return (Command) clazz.getMethod(methodName, params).invoke(null, args);
                } catch (Exception e) {
                    DriverStation.reportError(e.getMessage(), e.getStackTrace());
                    throw new IllegalArgumentException("Error constructing command");
                }
            }, params);
        }

        public static SpecializedNamedCommand fromClass(Class<?> clazz, Class<?>... params) {
            return new SpecializedNamedCommand(args -> {
                try {
                    return (Command) clazz.getConstructor(params).newInstance(args);
                } catch (Exception e) {
                    DriverStation.reportError(e.getMessage(), e.getStackTrace());
                    throw new IllegalArgumentException("Error constructing command");
                }
            }, params);
        }

        public static SpecializedNamedCommand empty(String name, Class<?>... params) {
            return new SpecializedNamedCommand(args -> Commands.none().withName(name));
        }

        public static SpecializedNamedCommand fromLambda(Function<Object, Command> fn, Class<?>... params) {
            return new SpecializedNamedCommand(
                    args -> fn.apply(args[0]),
                    params);
        }

        public static SpecializedNamedCommand fromLambda2(BiFunction<Object, Object, Command> fn, Class<?>... params) {
            return new SpecializedNamedCommand(
                    args -> fn.apply(args[0], args[1]),
                    params);
        }

        public static SpecializedNamedCommand fromLambda3(
                TriFunction<Object, Object, Object, Command> fn,
                Class<?>... params) {
            return new SpecializedNamedCommand(
                    args -> fn.apply(args[0], args[1], args[2]),
                    params);
        }
    }

    public record DefaultableNamedCommand(String name) {
        public void withDefault(Object... args) {
            NamedCommands.registerCommand(name, commands.get(name).construct(args));
        }
    }

    public static DefaultableNamedCommand registerCommand(String name, SpecializedNamedCommand command) {
        commands.put(name, command);
        return new DefaultableNamedCommand(name);
    }

    public static DefaultableNamedCommand registerEmptyCommand(String name, Class<?>... params) {
        return registerCommand(name, SpecializedNamedCommand.empty(name, params));
    }

    public static void generateSpecialized() {
        Path pathPlannerDir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner");
        File pathsDir = new File(pathPlannerDir.toFile(), "paths");
        File autosDir = new File(pathPlannerDir.toFile(), "autos");
        ArrayList<File> files = new ArrayList<>();
        files.addAll(List.of(pathsDir.listFiles()));
        files.addAll(List.of(autosDir.listFiles()));

        for (File file : files) {
            String contents = "";
            try {
                contents = Files.readString(file.toPath());
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), true);
            }

            /** Gets the all command expressions from a file string */
            final Pattern commandPattern = Pattern.compile("\"name\": \"(.*\\(.*)\"");
            /** Groups all the arguments from one command expression */
            final Pattern argsPattern = Pattern.compile("(true|false|\\d|\\.+)+");

            Matcher commandMatcher = commandPattern.matcher(contents);

            ArrayList<String> commandExpressions = new ArrayList<>();
            while (commandMatcher.find()) {
                commandExpressions.add(
                        commandMatcher.group()
                                .replace("\"name\": \"", "")
                                .replace("\"", ""));
            }

            for (String commandExpression : commandExpressions) {
                Matcher argsMatcher = argsPattern.matcher(commandExpression);
                ArrayList<String> args = new ArrayList<>();
                while (argsMatcher.find()) {
                    args.add(argsMatcher.group());
                }
                String commandName = commandExpression
                        .split("\\(")[0];
                if (commands.containsKey(commandName) && !NamedCommands.hasCommand(commandExpression)) {
                    NamedCommands.registerCommand(commandExpression,
                            commands.get(commandName).construct(args.toArray(new String[0])));
                } else {
                    DriverStation.reportError("Command not found: " + commandName, false);
                }
            }
        }

        commands.clear();
    }
}
