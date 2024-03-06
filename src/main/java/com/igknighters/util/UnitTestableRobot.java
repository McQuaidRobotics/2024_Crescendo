package com.igknighters.util;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import com.igknighters.GlobalState;
import com.igknighters.SubsystemResources.AllSubsystems;
import com.igknighters.constants.ConstValues;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import monologue.MonologueDashboard;

public class UnitTestableRobot extends TimedRobot {
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

    public UnitTestableRobot() {
        super(ConstValues.PERIODIC_TIME);
        isUnitTest = GlobalState.isUnitTest();

        if (isUnitTest) {
            DriverStationSim.setEnabled(false);
            DriverStationSim.setAutonomous(false);
            DriverStationSim.setTest(false);
        }
    }

    private final boolean isUnitTest;

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
    }

    @Override
    protected void loopFunc() {
        Tracer.startTrace("RobotLoop");
        if (isUnitTest) {
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

        MonologueDashboard.put("RobotMode", mode.toString());

        if ((!calledDsConnected && DriverStation.isDSAttached()) || isUnitTest) {
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
        if (this.isUnitTest)
            this.disabledInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withAutonomousInitTest(Consumer<UnitTestableRobot> testInit) {
        if (this.isUnitTest)
            this.autonomousInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withTeleopInitTest(Consumer<UnitTestableRobot> testInit) {
        if (this.isUnitTest)
            this.teleopInitTest = Optional.of(testInit);
        return this;
    }

    public UnitTestableRobot withDisablePeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (this.isUnitTest)
            this.disabledPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withAutonomousPeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (this.isUnitTest)
            this.autonomousPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withTeleopPeriodicTest(Consumer<UnitTestableRobot> testPeriodic) {
        if (this.isUnitTest)
            this.teleopPeriodicTest = Optional.of(testPeriodic);
        return this;
    }

    public UnitTestableRobot withDisableExitTest(Consumer<UnitTestableRobot> testExit) {
        if (this.isUnitTest)
            this.disabledExitTest = Optional.of(testExit);
        return this;
    }

    public UnitTestableRobot withAutonomousExitTest(Consumer<UnitTestableRobot> testExit) {
        if (this.isUnitTest)
            this.autonomousExitTest = Optional.of(testExit);
        return this;
    }

    public UnitTestableRobot withTeleopExitTest(Consumer<UnitTestableRobot> testExit) {
        if (this.isUnitTest)
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

    public AllSubsystems getAllSubsystemsForTest() {
        return new AllSubsystems();
    }
}
