package com.igknighters;

import java.util.HashMap;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.MonoDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Monologue;
import monologue.Monologue.MonologueConfig;

import com.igknighters.commands.autos.AutosCmdRegister;
import com.igknighters.commands.swerve.teleop.TeleopSwerveBase;
import com.igknighters.commands.umbrella.UmbrellaCommands;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstantHelper;
import com.igknighters.constants.RobotSetup;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.DriverController;
import com.igknighters.controllers.OperatorController;
import com.igknighters.controllers.TestingController;
import com.igknighters.subsystems.SubsystemResources.AllSubsystems;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.ShuffleboardApi;
import com.igknighters.util.Tracer;
import com.igknighters.util.can.CANBusLogging;
import com.igknighters.util.can.CANSignalManager;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.robots.UnitTestableRobot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class Robot extends UnitTestableRobot implements Logged {

    private Command autoCmd;
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final DriverController driverController;
    private final OperatorController operatorController;
    private final TestingController testingController;
    private AllSubsystems allSubsystems;

    public Robot() {
        super(ConstValues.PERIODIC_TIME);
        if (isUnitTest()) GlobalState.restoreDefaultState();

        setupLogging();

        driverController = new DriverController(0);
        operatorController = new OperatorController(1);
        testingController = new TestingController(3, true);
    }

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());

        ConstantHelper.applyRoboConst(ConstValues.class);

        GlobalState.publishField2d();

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
            var swerve = allSubsystems.swerve.get();

            swerve.setDefaultCommand(new TeleopSwerveBase.TeleopSwerveOmni(swerve, driverController));

            setupAutos(swerve);
        }

        if (allSubsystems.umbrella.isPresent()) {
            var umbrella = allSubsystems.umbrella.get();
            umbrella.setDefaultCommand(
                UmbrellaCommands.idleShooter(umbrella)
            );
        }

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
        Tracer.traceFunc("GlobalStateLog", GlobalState::log);
        Tracer.endTrace();
    }

    @Override
    public void disabledPeriodic() {
        autoCmd = GlobalState.getAutoCommand();
        MonoDashboard.put("Commands/SelectedAutoCommand", autoCmd.getName());
    }

    @Override
    public void autonomousInit() {
        if (isUnitTest()) {
            autoCmd = GlobalState.getAutoCommand();
        }
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
        if (!isUnitTest()) {
            Monologue.setupMonologue(
                this,
                "/Robot",
                new MonologueConfig()
                    .withDatalogPrefix("")
                    .withFileOnly(DriverStation::isFMSAttached)
                    .withLazyLogging(true)
            );
        } else {
            Monologue.setupMonologueDisabled(this, "/Robot", true);
        }

        Monologue.sendNetworkToFile("/Visualizers");
        Monologue.sendNetworkToFile("/PathPlanner");
        Monologue.sendNetworkToFile("/Tunables");
        Monologue.sendNetworkToFile("/Tracer");

        final String meta = "/Metadata/";
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

    private void setupAutos(Swerve swerve) {

        if (AutoBuilder.isConfigured() && isUnitTest()) {
            // this code can be run multiple times during unit tests,
            // because of AutoBuilder once paradigm this causes a crash
            return;
        }

        AutosCmdRegister.registerCommands(allSubsystems);

        AutoBuilder.configureHolonomic(
                swerve::getPose,
                swerve::resetOdometry,
                swerve::getChassisSpeed,
                chassisSpeeds -> swerve.drive(
                        chassisSpeeds, false),
                new HolonomicPathFollowerConfig(
                        kAuto.AUTO_TRANSLATION_PID,
                        kAuto.AUTO_ANGULAR_PID,
                        kSwerve.MAX_DRIVE_VELOCITY,
                        kSwerve.DRIVEBASE_RADIUS,
                        kAuto.DYNAMIC_REPLANNING_CONFIG),
                AllianceFlip::isRed,
                swerve);

        GlobalState.onceInitAutoChooser(swerve);
    }

    public AllSubsystems getAllSubsystems() {
        if (!isUnitTest()) {
            throw new RuntimeException("This method should only be called in unit tests");
        }
        return allSubsystems;
    }
}
