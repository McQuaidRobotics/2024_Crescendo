package com.igknighters;

import java.util.HashMap;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.MonoDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;

import com.ctre.phoenix6.SignalLogger;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBase;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstantHelper;
import com.igknighters.constants.RobotSetup;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.PowerLogger;
import com.igknighters.util.ShuffleboardApi;
import com.igknighters.util.Tracer;
import com.igknighters.util.can.CANBusLogging;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.robots.UnitTestableRobot;

public class Robot extends UnitTestableRobot<Robot> implements Logged {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final PowerLogger powerLogger = new PowerLogger(
        ConstValues.PDH_CAN_ID,
        ModuleType.kRev,
        "/PowerDistribution",
        true
    );;

    public final Localizer localizer = new Localizer();

    private final DriverController driverController = new DriverController(0, localizer);;
    private final OperatorController operatorController = new OperatorController(1);
    private final TestingController testingController = new TestingController(3, true);

    public final AllSubsystems allSubsystems;

    private Command autoCmd;


    public Robot() {
        super(ConstValues.PERIODIC_TIME);

        // logging needs to be setup asap as to not lose logging calls b4 its setup
        setupLogging();

        // consts also should be early initialized
        ConstantHelper.applyRoboConst(ConstValues.class);

        allSubsystems = new AllSubsystems(RobotSetup.getRobotID().subsystems);

        for (var subsystem : allSubsystems.getEnabledSubsystems()) {
            if (subsystem instanceof Logged) {
                Monologue.logObj((Logged) subsystem, "/Robot/" + subsystem.getName());
            }
        }

        driverController.assignButtons(allSubsystems);
        operatorController.assignButtons(allSubsystems);
        testingController.assignButtons(allSubsystems);

        if (allSubsystems.swerve.isPresent()) {
            final Swerve swerve = allSubsystems.swerve.get();

            localizer.resetOdometry(GeomUtil.POSE2D_CENTER, swerve.getModulePositions());

            swerve.setDefaultCommand(new TeleopSwerveBase.TeleopSwerveOmni(swerve, driverController, localizer));

            // setupAutos(swerve);
        }

        if (allSubsystems.umbrella.isPresent()) {
            var umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(
                UmbrellaCommands.idleShooter(umbrella)
            );
        }

        autoCmd = Commands.none().withName("Nothing Auto");

        System.gc();
    }

    @Override
    public void robotPeriodic() {
        Tracer.startTrace("RobotPeriodic");
        Tracer.traceFunc("CANSignalRefresh", CANSignalManager::refreshSignals);
        Tracer.traceFunc("CommandScheduler", scheduler::run);
        Tracer.traceFunc("LEDUpdate", LED::run);
        Tracer.traceFunc("CANBusLoggung", CANBusLogging::run);
        Tracer.traceFunc("Shuffleboard", ShuffleboardApi::run);
        Tracer.traceFunc("Monologue", Monologue::updateAll);
        Tracer.traceFunc("PowerLogger", powerLogger::log);
        Tracer.endTrace();
    }

    @Override
    public void disabledPeriodic() {
        MonoDashboard.put("Commands/SelectedAutoCommand", autoCmd.getName());
    }

    @Override
    public void autonomousInit() {
        if (autoCmd != null) {
            MonoDashboard.put("Commands/CurrentAutoCommand", autoCmd.getName());
            System.out.println("---- Starting auto command: " + autoCmd.getName() + " ----");
            scheduler.schedule(autoCmd);
        }
    }

    @Override
    public void autonomousExit() {
        if (autoCmd != null) {
            MonoDashboard.put("Commands/CurrentAutoCommand", "");
            autoCmd.cancel();
        }
        System.gc();
    }

    private void setupLogging() {
        // turn off auto logging for signal logger, doesnt get us any info we need
        if (isReal()) SignalLogger.enableAutoLogging(false);

        if (!isUnitTest()) {
            // setup monologue with lazy logging and no datalog prefix
            // robot is the root object
            Monologue.setupMonologue(
                this,
                "/Robot",
                new MonologueConfig()
                    .withDatalogPrefix("")
                    .withFileOnly(DriverStation::isFMSAttached)
                    .withLazyLogging(true)
            );
        } else {
            // used for tests and CI, does not actually log anything but asserts the logging is setup mostly correct
            Monologue.setupMonologueDisabled(this, "/Robot", true);
        }

        // send all library/utility made tables that don't use monologue to file aswell
        Monologue.sendNetworkToFile("/Visualizers");
        Monologue.sendNetworkToFile("/PathPlanner");
        Monologue.sendNetworkToFile("/Tunables");
        Monologue.sendNetworkToFile("/Tracer");
        Monologue.sendNetworkToFile("/PowerDistribution");

        // logs build data to the datalog
        final String meta = "/BuildData/";
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
}
