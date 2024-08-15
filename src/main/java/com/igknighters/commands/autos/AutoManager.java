package com.igknighters.commands.autos;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import com.igknighters.Robot;

import choreo.ChoreoAutoFactory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

/**
 * An auto chooser that allows for the selection of auto routines at runtime.
 */
public class AutoManager implements Logged {
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

    @Log(key = "selected")
    private String lastAutoRoutineName = NONE_NAME;
    private Command lastAutoRoutine = AutoRoutineGenerator.NONE.apply(null);

    public AutoManager(ChoreoAutoFactory factory) {
        this.factory = factory;
        Monologue.log(path + "/.type", "String Chooser");
        Monologue.log(path + "/default", NONE_NAME);
        Monologue.log(path + "/active", NONE_NAME);
        Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));

        if (Robot.isSimulation()) {
            new Trigger(DriverStation::isAutonomousEnabled).onTrue(
                Commands.waitSeconds(15.3).andThen(() -> DriverStationSim.setEnabled(false))
            );
        }
    }

    public void update() {
        if ((DriverStation.isDisabled() || Robot.isSimulation()) && DriverStation.isDSAttached() && !Robot.isUnitTest()) {
            if (selected.get().equals(lastAutoRoutineName)) return;
            lastAutoRoutineName = selected.get();
            lastAutoRoutine = autoRoutines.get(lastAutoRoutineName)
                .apply(this.factory);
            Monologue.log(path + "/active", lastAutoRoutineName);
        }
    }

    public void addAutoRoutine(String name, AutoRoutineGenerator generator) {
        autoRoutines.put(name, generator);
        Monologue.log(path + "/options", autoRoutines.keySet().toArray(new String[0]));
    }

    public void choose(String choice) {
        if (Robot.isUnitTest()) {
            lastAutoRoutineName = choice;
            lastAutoRoutine = autoRoutines.get(choice).apply(factory);
        } else {
            selected.set(choice);
            update();
        }
    }

    public Command getSelectedAutoRoutine() {
        return lastAutoRoutine;
    }
}
