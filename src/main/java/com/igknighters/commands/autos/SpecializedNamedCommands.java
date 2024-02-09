package com.igknighters.commands.autos;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

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

        public static SpecializedNamedCommand fromMethod(Class<?> clazz, String methodName, Class<?>... params) {
            return new SpecializedNamedCommand(args -> {
                try {
                    return (Command) clazz.getMethod(methodName, params).invoke(null, args);
                } catch (Exception e) {
                    DriverStation.reportError(e.getMessage(), true);
                    throw new IllegalArgumentException("Error constructing command");
                }
            }, params);
        }

        public static SpecializedNamedCommand fromClass(Class<?> clazz, Class<?>... params) {
            return new SpecializedNamedCommand(args -> {
                try {
                    return (Command) clazz.getConstructor(params).newInstance(args);
                } catch (Exception e) {
                    DriverStation.reportError(e.getMessage(), true);
                    throw new IllegalArgumentException("Error constructing command");
                }
            }, params);
        }
    }

    public static void registerCommand(String name, SpecializedNamedCommand command) {
        commands.put(name, command);
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
            String[] lines = contents.split("\n");
            for (String line : lines) {
                if (line.contains("\"name\": ") && line.contains("(") && line.contains(")")) {
                    boolean hasCommandName = false;
                    for (String valid : commands.keySet()) {
                        if (line.contains(valid)) {
                            hasCommandName = true;
                            break;
                        }
                    }
                    if (!hasCommandName) {
                        continue;
                    }
                    String trimmedLine = line.split("\"name\": ")[1].trim().replace(")", "").replace("\"", "");
                    String commandName = trimmedLine.split("\\(")[0].trim();
                    String[] args = trimmedLine.split("\\(")[1].split(",");
                    for (int i = 0; i < args.length; i++) {
                        args[i] = args[i].trim();
                    }
                    if (commands.containsKey(commandName)) {
                        NamedCommands.registerCommand(commandName, commands.get(commandName).construct(args));
                    } else {
                        DriverStation.reportError("Command not found: " + commandName, false);
                    }
                }
            }
        }

        commands.clear();
    }
}
