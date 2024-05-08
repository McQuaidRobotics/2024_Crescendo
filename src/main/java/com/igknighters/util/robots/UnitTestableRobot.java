package com.igknighters.util.robots;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.igknighters.util.Tracer;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A way to run your *whole* robot code in a unit test.
 * 
 * This allows you to hook onto to any of the robot mode functions and run your own code.
 * 
 * This also allows you to interupt the robot code in a way without the unit test thinking it crashed.
 * 
 * <pre><code>
 * 
 *  @Test
 *  public void testAuto(@Robo Robot robot) {
 *    RobotSetup.testOverrideRobotID(RobotID.SIM_CRASH);
 *    Pose2d desiredEndPose = new Pose2d(
 *        new Translation2d(3.0, 7.0),
 *        new Rotation2d());

 *    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

 *    // meters
 *    final double translationTolerance = 0.2;

 *    Autos.setAutoOverrideTest(new ProxyCommand(() -> new PathPlannerAuto("1 Meter Auto")));
 *    DriverStationSim.setAutonomous(true);
 *    DriverStationSim.setEnabled(true);

 *    robot.withAutonomousPeriodicTest(robo -> {
 *        boolean isFinished = GlobalState.getLocalizedPose()
 *          .getTranslation()
 *          .getDistance(desiredEndPose.getTranslation()) < translationTolerance;

 *        if (isFinished) {
 *            robo.finishUnitTestRobot();
 *        } else if (robo.getElapsedTime() > 2.5) {
 *            throw new RuntimeException(
 *                "Auto took to long, ended at "
 *                    + GlobalState.getLocalizedPose().toString());
 *        }
 *    });

 *    robot.runTest(3);

 *    System.gc();
 *  }
 * 
 * </code></pre>
 * 
 */
public class UnitTestableRobot extends TimedRobot {
    private static Optional<Boolean> cachedIsTest = Optional.empty();
    public static boolean isUnitTest() {
        if (cachedIsTest.isEmpty()) {
            cachedIsTest = Optional.of(System.getenv("test") != null);
        }
        return cachedIsTest.get();
    }

    public static class UnitTestableRobotExited extends RuntimeException {
        private static final long serialVersionUID = 1L;

        public UnitTestableRobotExited() {
            super("Unit test robot exited properly");
        }
    }

    public static class UnitTestableRobotTimedOut extends RuntimeException {
        private static final long serialVersionUID = 1L;

        public UnitTestableRobotTimedOut() {
            super("Unit test robot timed out");
        }
    }

    public static enum Mode {
        kNone,
        kDisabled,
        kAutonomous,
        kTeleop,
        kTest
    }

    private AtomicBoolean killswitch = new AtomicBoolean(false);

    private final Timer timer = new Timer();
    private double timeoutDuration = 30.0;

    private Mode lastMode = Mode.kNone;
    private boolean calledDsConnected = false;
    private int loopCount = 0;

    private Optional<Consumer<UnitTestableRobot>> disabledInitTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> autonomousInitTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> teleopInitTest = Optional.empty();

    private Optional<Consumer<UnitTestableRobot>> disabledPeriodicTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> autonomousPeriodicTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> teleopPeriodicTest = Optional.empty();

    private Optional<Consumer<UnitTestableRobot>> disabledExitTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> autonomousExitTest = Optional.empty();
    private Optional<Consumer<UnitTestableRobot>> teleopExitTest = Optional.empty();

    public UnitTestableRobot(double period) {
        super(
            new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    if (isUnitTest()) assert HAL.initialize(500, 0);
                    return period;
                }
            }.getAsDouble()
        );
        System.out.println("isUnitTest: " + isUnitTest());
        if (isUnitTest()) {
            DriverStationSim.setEnabled(false);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(false);
            DriverStationSim.setDsAttached(true);
        }
    }

    public void finishUnitTestRobot() {
        killswitch.set(true);
    }

    public void runTest(double timeout) {
        this.timeoutDuration = timeout;
        timer.start();
        try {
            this.startCompetition();
        } catch (UnitTestableRobotExited e) {
            // Expected
        }
        this.endCompetition();
        this.close();
        if (isUnitTest()) {
            HAL.exitMain();
            HAL.shutdown();
            var cmdScheduler = CommandScheduler.getInstance();
            cmdScheduler.cancelAll();
            cmdScheduler.getActiveButtonLoop().clear();
            cmdScheduler.getDefaultButtonLoop().clear();
            cmdScheduler.clearComposedCommands();
            cmdScheduler.unregisterAllSubsystems();
        }
    }

    @Override
    protected void loopFunc() {
        Tracer.startTrace("RobotLoop");
        if (isUnitTest()) {
            if (killswitch.get()) {
                throw new UnitTestableRobotExited();
            }
            if (timer.hasElapsed(timeoutDuration)) {
                throw new UnitTestableRobotTimedOut();
            }
        }
        DriverStation.refreshData();
        // Get current mode
        Mode mode = Mode.kNone;
        if (DriverStation.isDisabled()) {
            mode = Mode.kDisabled;
        } else if (DriverStation.isAutonomous()) {
            mode = Mode.kAutonomous;
        } else if (DriverStation.isTeleop()) {
            mode = Mode.kTeleop;
        } else if (DriverStation.isTest()) {
            mode = Mode.kTest;
        }

        if ((!calledDsConnected && DriverStation.isDSAttached()) || isUnitTest()) {
            calledDsConnected = true;
            driverStationConnected();
        }

        // If mode changed, call mode exit and entry functions
        if (lastMode != mode) {
            // Call last mode's exit function
            if (lastMode == Mode.kDisabled) {
                disabledExit();
                disabledExitTest.ifPresent((test) -> test.accept(this));
            } else if (lastMode == Mode.kAutonomous) {
                autonomousExit();
                autonomousExitTest.ifPresent((test) -> test.accept(this));
            } else if (lastMode == Mode.kTeleop) {
                teleopExit();
                teleopExitTest.ifPresent((test) -> test.accept(this));
            } else if (lastMode == Mode.kTest) {
                testExit();
                teleopExitTest.ifPresent((test) -> test.accept(this));
            }

            // Call current mode's entry function
            if (mode == Mode.kDisabled) {
                disabledInit();
                disabledInitTest.ifPresent((test) -> test.accept(this));
            } else if (mode == Mode.kAutonomous) {
                autonomousInit();
                autonomousInitTest.ifPresent((test) -> test.accept(this));
            } else if (mode == Mode.kTeleop) {
                teleopInit();
                teleopInitTest.ifPresent((test) -> test.accept(this));
            } else if (mode == Mode.kTest) {
                testInit();
                teleopInitTest.ifPresent((test) -> test.accept(this));
            }

            lastMode = mode;
        }

        // Call the appropriate function depending upon the current robot mode
        if (mode == Mode.kDisabled) {
            DriverStationJNI.observeUserProgramDisabled();
            Tracer.traceFunc("disabledPeriodic", this::disabledPeriodic);
            disabledPeriodicTest.ifPresent((test) -> test.accept(this));
        } else if (mode == Mode.kAutonomous) {
            DriverStationJNI.observeUserProgramAutonomous();
            Tracer.traceFunc("autonomousPeriodic", this::autonomousPeriodic);
            autonomousPeriodicTest.ifPresent((test) -> test.accept(this));
        } else if (mode == Mode.kTeleop) {
            DriverStationJNI.observeUserProgramTeleop();
            Tracer.traceFunc("teleopPeriodic", this::teleopPeriodic);
            teleopPeriodicTest.ifPresent((test) -> test.accept(this));
        } else {
            DriverStationJNI.observeUserProgramTest();
            Tracer.traceFunc("testPeriodic", this::testPeriodic);
            teleopPeriodicTest.ifPresent((test) -> test.accept(this));
        }

        Tracer.traceFunc("robotPeriodic", this::robotPeriodic);

        SmartDashboard.updateValues();

        if (isSimulation()) {
            HAL.simPeriodicBefore();
            simulationPeriodic();
            HAL.simPeriodicAfter();
        }

        NetworkTableInstance.getDefault().flushLocal();

        Tracer.endTrace();
        loopCount++;
    }

    public UnitTestableRobot withDisableInitTest(Consumer<UnitTestableRobot> testInit) {
        if (isUnitTest())
            this.disabledInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withAutonomousInitTest(Consumer<UnitTestableRobot> testInit) {
        if (isUnitTest())
            this.autonomousInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withTeleopInitTest(Consumer<UnitTestableRobot> testInit) {
        if (isUnitTest())
            this.teleopInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withDisablePeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (isUnitTest())
            this.disabledPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withAutonomousPeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (isUnitTest())
            this.autonomousPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withTeleopPeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (isUnitTest())
            this.teleopPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withDisableExitTest(Consumer<UnitTestableRobot> testExit) {
        if (isUnitTest())
            this.disabledExitTest = Optional.of(testExit);
        return this;
    }

    public UnitTestableRobot withAutonomousExitTest(Consumer<UnitTestableRobot> testExit) {
        if (isUnitTest())
            this.autonomousExitTest = Optional.of(testExit);
        return this;
    }

    public UnitTestableRobot withTeleopExitTest(Consumer<UnitTestableRobot> testExit) {
        if (isUnitTest())
            this.teleopExitTest = Optional.of(testExit);
        return this;
    }

    public Mode getMode() {
        return lastMode;
    }

    public int getLoopCount() {
        return loopCount;
    }

    public double getElapsedTime() {
        return timer.get();
    }
}
