package com.igknighters.commands.autos;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import com.igknighters.Robot;

import choreo.ChoreoAutoFactory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Monologue;

/**
 * An auto chooser that allows for the selection of auto routines at runtime.
 */
public class AutoChooser {
    public static interface AutoRoutineGenerator extends Function<ChoreoAutoFactory, Command> {
        static final AutoRoutineGenerator NONE = factory -> Commands.none().withName("Do Nothing Auto");
    }

    private final String path = "/Auto Chooser";
    private final String NONE_NAME = "Nothing";

    private final HashMap<String, AutoRoutineGenerator> autoRoutines = new HashMap<>(
        Map.of(NONE_NAME, AutoRoutineGenerator.NONE)
    );

    private final StringEntry selected = NetworkTableInstance
        .getDefault()
        .getTable(path)
        .getStringTopic("selected")
        .getEntry(NONE_NAME);

    private final ChoreoAutoFactory factory;

    private String lastAutoRoutineName = NONE_NAME;
    private Command lastAutoRoutine = AutoRoutineGenerator.NONE.apply(null);

    public AutoChooser(ChoreoAutoFactory factory) {
        this.factory = factory;
        Monologue.log(path + "/.type", "String Chooser");
        Monologue.log(path + "/default", NONE_NAME);
        Monologue.log(path + "/active", NONE_NAME);
        Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));
    }

    public void update() {
        if (DriverStation.isDisabled() || Robot.isSimulation()) {
            if (selected.get().equals(lastAutoRoutineName)) return;
            lastAutoRoutineName = selected.get();
            lastAutoRoutine = autoRoutines.get(lastAutoRoutineName).apply(this.factory);
            Monologue.log(path + "/active", lastAutoRoutineName);
        }
    }

    public void addAutoRoutine(String name, AutoRoutineGenerator generator) {
        autoRoutines.put(name, generator);
        Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));
    }

    public void choose(String choice) {
        selected.set(choice);
        update();
    }

    public Command getSelectedAutoRoutine() {
        return lastAutoRoutine;
    }
}
