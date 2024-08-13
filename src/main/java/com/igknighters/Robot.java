package com.igknighters;

import java.util.HashMap;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.Monologue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Monologue.MonologueConfig;

import com.ctre.phoenix6.SignalLogger;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBase;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstantHelper;
import com.igknighters.constants.RobotConfig;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.FilesystemLogger;
import com.igknighters.util.GlobalField;
import com.igknighters.util.PowerLogger;
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
    );
    private final FilesystemLogger filesystemLogger = new FilesystemLogger();

    public final Localizer localizer = new Localizer();

    private final DriverController driverController = new DriverController(0, localizer);;
    private final OperatorController operatorController = new OperatorController(1);
    private final TestingController testingController = new TestingController(3);

    public final AllSubsystems allSubsystems;

    public Robot() {
        super(ConstValues.PERIODIC_TIME);

        // logging needs to be setup asap as to not lose logging calls b4 its setup
        setupLogging();

        // consts also should be early initialized
        ConstantHelper.applyRoboConst(ConstValues.class);

        allSubsystems = new AllSubsystems(RobotConfig.getRobotID().subsystems);

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
        }

        if (allSubsystems.umbrella.isPresent()) {
            var umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(
                    UmbrellaCommands.idleShooter(umbrella));
        }

        System.gc();
    }

    @Override
    public void robotPeriodic() {
        Tracer.startTrace("RobotPeriodic");
        Tracer.traceFunc("CANSignalRefresh", CANSignalManager::refreshSignals);
        Tracer.traceFunc("Localizer", localizer::update);
        Tracer.traceFunc("CommandScheduler", scheduler::run);
        Tracer.traceFunc("LEDUpdate", LED::run);
        Tracer.traceFunc("CANBusLoggung", CANBusLogging::log);
        Tracer.traceFunc("PowerLogger", powerLogger::log);
        Tracer.traceFunc("FilesystemLogger", filesystemLogger::log);
        Tracer.traceFunc("Monologue", Monologue::updateAll);
        Tracer.endTrace();
    }

    @Override
    public void disabledPeriodic() {
        // Monologue.log("Commands/SelectedAutoCommand", autoCmd.getName());
    }

    @Override
    public void autonomousInit() {
        // if (autoCmd != null) {
        //     Monologue.log("Commands/CurrentAutoCommand", autoCmd.getName());
        //     System.out.println("---- Starting auto command: " + autoCmd.getName() + " ----");
        //     scheduler.schedule(autoCmd);
        // }
    }

    @Override
    public void autonomousExit() {
        // if (autoCmd != null) {
        //     Monologue.log("Commands/CurrentAutoCommand", "");
        //     autoCmd.cancel();
        // }
        System.gc();
    }

    private void setupLogging() {
        // turn off auto logging for signal logger, doesnt get us any info we need
        if (isReal()) {
            SignalLogger.enableAutoLogging(false);
            filesystemLogger.addFile("/home/lvuser/FRC_UserProgram.log");
            filesystemLogger.addFile("/var/log/dmesg");
        }

        if (!isUnitTest()) {
            // setup monologue with lazy logging and no datalog prefix
            // robot is the root object
            Monologue.setupMonologue(
                    this,
                    "/Robot",
                    new MonologueConfig()
                            .withDatalogPrefix("")
                            .withFileOnly(DriverStation::isFMSAttached)
                            .withLazyLogging(true));

            GlobalField.enable();
        } else {
            // used for tests and CI, does not actually log anything but asserts the logging is setup mostly correct
            Monologue.setupMonologueDisabled(this, "/Robot", true);
        }

        // send all library/utility made tables that don't use monologue to file aswell
        Monologue.sendNetworkToFile("/Visualizers");
        Monologue.sendNetworkToFile("/Tunables");
        Monologue.sendNetworkToFile("/Tracer");
        Monologue.sendNetworkToFile("/PowerDistribution");

        // logs build data to the datalog
        final String meta = "/BuildData/";
        Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
        Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
        Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
        Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
        Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
        Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Monologue.log(meta + "GitDirty", "All changes committed");
                break;
            case 1:
                Monologue.log(meta + "GitDirty", "Uncomitted changes");
                break;
            default:
                Monologue.log(meta + "GitDirty", "Unknown");
                break;
        }

        HashMap<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Monologue.log(
                    "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
                    active.booleanValue());
            Monologue.log("Commands/CommandsAll/" + name, count > 0);
        };
        scheduler.onCommandInitialize(
                (Command command) -> {
                    logCommandFunction.accept(command, true);
                });
        scheduler.onCommandFinish(
                (Command command) -> {
                    logCommandFunction.accept(command, false);
                });
        scheduler.onCommandInterrupt(
                (Command command) -> {
                    logCommandFunction.accept(command, false);
                });
    }

    public static boolean isDemo() {
        return ConstValues.DEMO;
    }

    public static boolean isDebug() {
        return ConstValues.DEBUG;
    }
}
