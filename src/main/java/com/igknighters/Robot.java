package com.igknighters;

import java.util.HashMap;
import java.util.function.BiConsumer;

import monologue.MonoDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Monologue;

import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.constants.ConstValues;
import com.igknighters.util.CANBusLogging;
import com.igknighters.util.ShuffleboardApi;
import com.igknighters.util.Tracer;
import com.igknighters.util.UnitTestableRobot;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class Robot extends UnitTestableRobot {

    private Command autoCmd;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private RobotContainer roboContainer;

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());
        setupLogging();

        com.igknighters.ConstantHelper.applyRoboConst(ConstValues.class);

        GlobalState.publishField2d();

        roboContainer = new RobotContainer();

        if (!GlobalState.isUnitTest()) {
            Monologue.setupMonologue(roboContainer, "/Robot", false, true);
            roboContainer.initMonologue();
        } else {
            Monologue.setupMonologueForUnitTest();
        }
    }

    @Override
    public void robotPeriodic() {
        Tracer.traceFunc("CommandScheduler", scheduler::run);
        Tracer.traceFunc("LEDUpdate", LED::run);
        Tracer.traceFunc("CANBusLoggung", CANBusLogging::run);
        Tracer.traceFunc("Shuffleboard", ShuffleboardApi::run);
        Tracer.traceFunc("Monologue", () -> {
            Monologue.updateAll(
                DriverStation.isFMSAttached()
            );
        });
        GlobalState.log();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        autoCmd = GlobalState.getAutoCommand();
        MonoDashboard.put("Commands/SelectedAutoCommand", autoCmd.getName());
    }

    @Override
    public void autonomousInit() {
        if (GlobalState.isUnitTest()) {
            autoCmd = GlobalState.getAutoCommand();
        }
        if (autoCmd != null) {
            MonoDashboard.put("Commands/CurrentAutoCommand", autoCmd.getName());
            System.out.println("---- Starting auto command: " + autoCmd.getName() + " ----");
            scheduler.schedule(autoCmd);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autoCmd != null) {
            MonoDashboard.put("Commands/CurrentAutoCommand", "");
            autoCmd.cancel();
        }
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void driverStationConnected() {
        if (DriverStation.isFMSAttached() && !GlobalState.isUnitTest()) {
            Monologue.setFileOnly(true);
        }
    }

    private void setupLogging() {
        if (GlobalState.isUnitTest()) {
            return;
        }

        final String meta = "Metadata/";
        MonoDashboard.put(meta + "RuntimeType", getRuntimeType().toString());
        MonoDashboard.put(meta + "ProjectName", BuildConstants.MAVEN_NAME);
        MonoDashboard.put(meta + "BuildDate", BuildConstants.BUILD_DATE);
        MonoDashboard.put(meta + "GitSHA", BuildConstants.GIT_SHA);
        MonoDashboard.put(meta + "GitDate", BuildConstants.GIT_DATE);
        MonoDashboard.put(meta + "GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                MonoDashboard.put(meta + "GitDirty", "All changes committed");
                break;
            case 1:
                MonoDashboard.put(meta + "GitDirty", "Uncomitted changes");
                break;
            default:
                MonoDashboard.put(meta + "GitDirty", "Unknown");
                break;
        }

        // if (Robot.isReal()) {
        // var path = "/media/sda1/";
        // if (!new java.io.File(path).exists() && ConstValues.DEBUG) {
        // DriverStation.reportError("DATALOGS USB NOT PLUGGED IN!!!", false);
        // } else {
        // Logger.addDataReceiver(
        // new ExtensibleWPILOGWriter(path)
        // .withNTPrefixListener("/Visualizers")
        // .withNTPrefixListener("/PathPlanner"));
        // }
        // }
        // Logger.addDataReceiver(new NT4Publisher());
        // Logger.start();

        HashMap<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            MonoDashboard.put(
                    "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
                    active.booleanValue());
            MonoDashboard.put("Commands/CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        (Command command) -> {
                            logCommandFunction.accept(command, true);
                        });
        CommandScheduler.getInstance()
                .onCommandFinish(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (Command command) -> {
                            logCommandFunction.accept(command, false);
                        });
    }

    @Override
    public AllSubsystems getAllSubsystemsForTest() {
        if (!GlobalState.isUnitTest()) {
            throw new RuntimeException("This method should only be called in unit tests");
        }
        return roboContainer.getAllSubsystemsForTest();
    }
}
