package com.igknighters;

import java.util.HashMap;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.Monologue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Monologue.MonologueConfig;

import com.ctre.phoenix6.SignalLogger;
import com.igknighters.commands.autos.AutoController;
import com.igknighters.commands.autos.AutoManager;
import com.igknighters.commands.autos.AutoRoutines;
import com.igknighters.commands.swerve.teleop.TeleopSwerveTraditionalCmd;
import com.igknighters.commands.tests.Characterizers;
import com.igknighters.commands.tests.TestManager;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstantHelper;
import com.igknighters.constants.RobotConfig;
import com.igknighters.constants.RobotConfig.RobotID;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.subsystems.umbrella.Umbrella;
import com.igknighters.util.can.CANBusLogging;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.logging.FilesystemLogger;
import com.igknighters.util.logging.GlobalField;
import com.igknighters.util.logging.PowerLogger;
import com.igknighters.util.logging.WatchdogSilencer;
import com.igknighters.util.logging.Tracer;
import com.igknighters.util.robots.UnitTestableRobot;

import choreo.Choreo;
import choreo.ChoreoAutoFactory.ChoreoAutoBindings;

public class Robot extends UnitTestableRobot<Robot> implements Logged {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final PowerLogger powerLogger = new PowerLogger(
            ConstValues.PDH_CAN_ID,
            ModuleType.kRev,
            "/Robot/PowerDistribution",
            false
    );
    private final FilesystemLogger filesystemLogger = new FilesystemLogger();

    public final Localizer localizer = new Localizer();

    private final DriverController driverController;
    private final OperatorController operatorController;
    private final TestingController testingController;

    public final AllSubsystems allSubsystems;

    public final AutoManager autoManager;
    public final TestManager testManager;

    public Robot() {
        this(null);
    }

    public Robot(RobotID robotID) {
        super(ConstValues.PERIODIC_TIME);

        // logging needs to be setup asap as to not lose logging calls b4 its setup
        setupLogging();

        if (robotID == null) {
            robotID = RobotConfig.getRobotID();
        }

        ConstantHelper.applyRoboConst(ConstValues.class, robotID);

        driverController = new DriverController(0, localizer);
        operatorController = new OperatorController(1);
        testingController = new TestingController(3);

        allSubsystems = new AllSubsystems(robotID.subsystems);

        for (final Logged subsystem : allSubsystems.getLoggableSubsystems()) {
            Monologue.logObj(subsystem, "/Robot/" + subsystem.getOverrideName());
        }

        driverController.assignButtons(allSubsystems);
        operatorController.assignButtons(allSubsystems);
        testingController.assignButtons(allSubsystems);

        if (allSubsystems.swerve.isPresent()) {
            final Swerve swerve = allSubsystems.swerve.get();

            localizer.resetOdometry(GeomUtil.POSE2D_CENTER, swerve.getModulePositions());

            swerve.setDefaultCommand(new TeleopSwerveTraditionalCmd(swerve, driverController));
        }

        if (allSubsystems.umbrella.isPresent()) {
            final Umbrella umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(
                    UmbrellaCommands.idleShooter(umbrella));

            umbrella.setupSimNoteDetection(localizer);
        }

        autoManager = new AutoManager(
            Choreo.createAutoFactory(
                allSubsystems.swerve.isPresent() ? allSubsystems.swerve.get() : new Subsystem() {},
                localizer::pose,
                new AutoController(),
                allSubsystems.swerve.isPresent()
                    ? chassisSpeeds -> allSubsystems.swerve.get().drive(chassisSpeeds, false)
                    : chassisSpeeds -> {},
                AllianceFlip::isRed,
                new ChoreoAutoBindings(),
                (traj, starting) -> {
                    String msg = "Auto Trajectory " + traj.name() + " " + (starting ? "Started" : "Finished");
                    System.out.println(msg);
                    Monologue.log("AutoEvent", msg);
                }
            )
        );

        if (allSubsystems.hasAllSubsystems()) {
            var routines = new AutoRoutines(allSubsystems, localizer);
            autoManager.addAutoRoutine("5 Piece Amp Side", routines::fivePieceAmpSide);
            autoManager.addAutoRoutine("6 Piece Amp Side Far", routines::sixPieceFarAmpSide);
            autoManager.addAutoRoutine("4 Piece Src Side", routines::fourPieceSourceSide);
            autoManager.addAutoRoutine("3 Piece Sub Middle", routines::threePieceSubMiddle);
        }

        testManager = new TestManager();

        if (allSubsystems.hasAllSubsystems()) {
            testManager.addTestRoutine(
                "Characterize Swerve",
                Characterizers.characterizeSwerve(allSubsystems.swerve.get())
            );
            testManager.addTestRoutine(
                "Characterize Pivot",
                Characterizers.characterizePivot(allSubsystems.stem.get())
            );
            testManager.addTestRoutine(
                "Characterize Wrist",
                Characterizers.characterizeWrist(allSubsystems.stem.get())
            );
            testManager.addTestRoutine(
                "Characterize Telescope",
                Characterizers.characterizeTelescope(allSubsystems.stem.get())
            );
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
        Tracer.traceFunc("AutoManager", autoManager::update);
        Tracer.endTrace();
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        System.gc();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        Command autoCmd = autoManager.getSelectedAutoRoutine();
        String msg = "---- Starting auto command: " + autoCmd.getName() + " ----";
        if (isDebug()) System.out.println(msg);
        Monologue.log("AutoEvent", msg);
        scheduler.schedule(autoCmd);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        scheduler.cancelAll();
        System.gc();
    }

    @Override
    public void testInit() {
        CANSignalManager.setCharacterizationMode(true);
    }

    @Override
    public void testExit() {
        CANSignalManager.setCharacterizationMode(false);
        scheduler.cancelAll();
        System.gc();
    }

    @Override
    public void simulationPeriodic() {}

    private void setupLogging() {
        WatchdogSilencer.silence(this, "m_watchdog");
        WatchdogSilencer.silence(scheduler, "m_watchdog");

        // turn off auto logging for signal logger, doesnt get us any info we need
        if (isReal()) {
            SignalLogger.enableAutoLogging(false);
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

        filesystemLogger.addFile("/home/lvuser/FRC_UserProgram.log", "Console", 0.27);
        filesystemLogger.addFile("/var/log/dmesg", "Dmesg", 2.2);
        filesystemLogger.addFile("/var/log/messages", "Kernel", 1.4);

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
            // Monologue.log(
            //         "Commands/CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
            //         active.booleanValue());
            Monologue.log("Commands/" + name, count > 0);
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
