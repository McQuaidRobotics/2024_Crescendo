package frc.robot;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;
import frc.robot.util.ShuffleboardApi;

public class Robot extends LoggedRobot {

    private Command autoCmd;
    @SuppressWarnings("unused")
    private final RobotContainer robotContainer = new RobotContainer();
    private Autos.AutoRoutines autoRoutine;
    private Optional<Alliance> alliance = Optional.empty();
    private final SendableChooser<Autos.AutoRoutines> autoRoutineChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        setupAkit();


        Autos.AutoRoutines[] autoRoutines = Autos.AutoRoutines.values();
        for (Autos.AutoRoutines autoRoutine : autoRoutines) {
            if (autoRoutine == Autos.AutoRoutines.NOTHING) {
                autoRoutineChooser.setDefaultOption(autoRoutine.name(), autoRoutine);
                continue;
            }
            autoRoutineChooser.addOption(autoRoutine.name(), autoRoutine);
        }

        SmartDashboard.putString("AutoCommand", autoCmd == null ? "null" : autoCmd.getName());
        ShuffleboardApi.getTab("Autos")
            .addSendable("Autonomous Routine", autoRoutineChooser);
    }

    @Override
    public void robotPeriodic() {
        ShuffleboardApi.run();
        CommandScheduler.getInstance().run();
        LED.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        var selectedRoutine = autoRoutineChooser.getSelected();
        var currAlliance = DriverStation.getAlliance();
        if (currAlliance.isPresent() && alliance.isPresent()) {
            if (selectedRoutine != autoRoutine || alliance.get() != currAlliance.get()) {
                autoRoutine = selectedRoutine;
                alliance = currAlliance;
                autoCmd = autoRoutine.getCommand();
                SmartDashboard.putString("AutoCommand", autoCmd == null ? "null" : autoCmd.getName());
            }
        }
    }

    @Override
    public void autonomousInit() {
        if (autoCmd != null) {
            autoCmd.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
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

    private void setupAkit() {
        Logger.recordMetadata("Robot", "CitrusCircus");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
        case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
        case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
        default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/robotlogs/"));
        }
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        HashMap<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
            (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
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
}
