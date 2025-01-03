package igknighters;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.LogSink;
import monologue.Monologue;
import monologue.Annotations.FlattenedLogged;
import monologue.Annotations.Log;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Monologue.MonologueConfig;

import com.ctre.phoenix6.SignalLogger;
import igknighters.commands.autos.AutoController;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.swerve.teleop.TeleopSwerveTraditionalCmd;
import igknighters.commands.tests.Characterizers;
import igknighters.commands.tests.TestManager;
import igknighters.commands.umbrella.UmbrellaCommands;
import igknighters.constants.ConstValues;
import igknighters.constants.FieldConstants;
import igknighters.constants.RobotConfig;
import igknighters.controllers.DriverController;
import igknighters.subsystems.SubsystemResources.AllSubsystems;
import igknighters.subsystems.swerve.Swerve;
import igknighters.subsystems.umbrella.Umbrella;
import igknighters.util.UnitTestableRobot;
import igknighters.util.can.CANSignalManager;
import igknighters.util.logging.WatchdogSilencer;
import igknighters.util.logging.Tracer;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;

public class Robot extends UnitTestableRobot<Robot> implements Logged {

    private static final AtomicInteger loopCount = new AtomicInteger(0);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @Log(key = "Localizer")
    public final Localizer localizer = new Localizer();
    public final SimCtx simCtx = new SimCtx(localizer, isSimulation());

    private final DriverController driverController;

    @FlattenedLogged
    public final AllSubsystems allSubsystems;

    public final AutoChooser autoChooser = new AutoChooser();
    public final TestManager testManager;


    public Robot() {
        super(ConstValues.PERIODIC_TIME);

        setupLogging();

        localizer.publishField();

        driverController = new DriverController(0, localizer);

        allSubsystems = new AllSubsystems(localizer, simCtx, RobotConfig.getRobotID().subsystems);

        driverController.assignButtons(allSubsystems);

        if (allSubsystems.swerve.isPresent()) {
            final Swerve swerve = allSubsystems.swerve.get();

            localizer.reset(FieldConstants.POSE2D_CENTER);

            swerve.setDefaultCommand(new TeleopSwerveTraditionalCmd(swerve, driverController));
        }

        if (allSubsystems.umbrella.isPresent()) {
            final Umbrella umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(UmbrellaCommands.idleShooter(umbrella, UmbrellaCommands::defaultIdleRPM));

            umbrella.setupSimNoteDetection(localizer);
        }

        final AutoFactory autoFactory = new AutoFactory(
            localizer::pose,
            localizer::reset,
            new AutoController(allSubsystems.swerve, localizer),
            true,
            allSubsystems.swerve.isPresent() ? allSubsystems.swerve.get() : new Subsystem() {},
            new AutoBindings(),
            (traj, starting) -> {
                String msg = "[Auto] Trajectory " + traj.name() + " " + (starting ? "Started" : "Finished");
                System.out.println(msg);
                Monologue.log("AutoEvent", msg);
                if (starting) {
                    Monologue.log("Auto/Trajectory", traj.samples().toArray(SwerveSample[]::new));
                } else {
                    Monologue.log("Auto/Trajectory", new SwerveSample[0]);
                }
            }
        );

        if (allSubsystems.hasAllSubsystems()) {
            final var routines = new AutoRoutines(allSubsystems, localizer, autoFactory);
            autoChooser.addRoutine("5 Piece Amp Side", routines::fivePieceAmpSide);
            autoChooser.addRoutine("6 Piece Amp Side Far", routines::sixPieceFarAmpSide);
            autoChooser.addRoutine("4 Piece Src Side", routines::fourPieceSourceSide);
        }
        setupAutoChooser();

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
        loopCount.incrementAndGet();
        Tracer.traceFunc("SimCtx", simCtx::update);
        Tracer.traceFunc("CANSignalRefresh", CANSignalManager::refreshSignals);
        Tracer.traceFunc("Localizer", localizer::update);
        Tracer.traceFunc("CommandScheduler", scheduler::run);
        Tracer.traceFunc("Monologue", Monologue::updateAll);
        Tracer.traceFunc("Choosers", () -> {
            testManager.update();
        });
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        System.gc();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void autonomousInit() {
        Command autoCmd = autoChooser.selectedCommand();
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
        testManager.getSelectedTestRoutine().schedule();
    }

    @Override
    public void testExit() {
        scheduler.cancelAll();
        System.gc();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void simulationPeriodic() {}

    private void setupLogging() {
        WatchdogSilencer.silence(this, "m_watchdog");
        WatchdogSilencer.silence(scheduler, "m_watchdog");

        DriverStation.silenceJoystickConnectionWarning(true);

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
                            .withOptimizeBandwidth(DriverStation::isFMSAttached)
                            .withLazyLogging(true));
        } else {
            // used for tests and CI, does not actually log anything but asserts the logging is setup mostly correct
            Monologue.setupMonologueDisabled(this, "/Robot", true);
        }

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

    private void setupAutoChooser() {
        Monologue.publishSendable("/Choosers/AutoChooser", autoChooser, LogSink.NT);
        final StringSubscriber sub = NetworkTableInstance.getDefault()
                .getStringTopic("/Choosers/AutoChooser/selected")
                .subscribe(
                    "", 
                    PubSubOption.pollStorage(1),
                    PubSubOption.periodic(0.5),
                    PubSubOption.sendAll(true),
                    PubSubOption.keepDuplicates(false)
                );
        this.addPeriodic(
            () -> {
                var queue = sub.readQueueValues();
                if (queue.length > 0) {
                    System.out.println("AutoChooser selected: " + queue[0]);
                    autoChooser.select(queue[0]);
                }
            },
            kDefaultPeriod,
            0.01
        );
    }

    public static int loopCount() {
        return loopCount.get();
    }

    public static boolean isDemo() {
        return ConstValues.DEMO;
    }

    public static boolean isDebug() {
        return ConstValues.DEBUG;
    }

    public static boolean isSunlight() {
        return ConstValues.SUNLIGHT;
    }
}