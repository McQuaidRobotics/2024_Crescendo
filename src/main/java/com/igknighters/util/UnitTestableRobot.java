package com.igknighters.util;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UnitTestableRobot extends LoggedRobot {

    private AtomicBoolean killswitch = new AtomicBoolean(false);

    public void killUnitTestRobot() {
        killswitch.set(true);
    }

    public static class UnitTestableRobotExited extends RuntimeException {
        private static final long serialVersionUID = 1L;

        public UnitTestableRobotExited() {
            super("Unit test robot exited properly");
        }
    }

    public static <R extends UnitTestableRobot> void testRobot(
            Supplier<R> robotSupplier,
            Consumer<UnitTestableRobot> testFn) {
        UnitTestableRobot robot = robotSupplier.get();
        robot.setTestFn(testFn);
        try {
            robot.startCompetition();
        } catch (UnitTestableRobotExited e) {
            // Expected
        }
        robot.endCompetition();
        robot.close();
    }

    private Optional<Consumer<UnitTestableRobot>> testFn = Optional.empty();

    private void setTestFn(Consumer<UnitTestableRobot> testFn) {
        this.testFn = Optional.of(testFn);
    }

    public UnitTestableRobot() {
        super();
    }

    @Override
    protected void loopFunc() {
        Tracer.traceFunc("RobotLoop", this::innerLoop);
        testFn.ifPresent(fn -> fn.accept(this));
    }

    private enum Mode {
        kNone,
        kDisabled,
        kAutonomous,
        kTeleop,
        kTest
    }

    private Mode lastMode = Mode.kNone;
    private boolean calledDsConnected = false;

    private void innerLoop() {
        if (killswitch.get()) {
            throw new UnitTestableRobotExited();
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

        Logger.recordOutput("RobotMode", mode.toString());

        if (!calledDsConnected && DriverStation.isDSAttached()) {
            calledDsConnected = true;
            driverStationConnected();
        }

        // If mode changed, call mode exit and entry functions
        if (lastMode != mode) {
            // Call last mode's exit function
            if (lastMode == Mode.kDisabled) {
                disabledExit();
            } else if (lastMode == Mode.kAutonomous) {
                autonomousExit();
            } else if (lastMode == Mode.kTeleop) {
                teleopExit();
            } else if (lastMode == Mode.kTest) {
                testExit();
            }

            // Call current mode's entry function
            if (mode == Mode.kDisabled) {
                disabledInit();
            } else if (mode == Mode.kAutonomous) {
                autonomousInit();
            } else if (mode == Mode.kTeleop) {
                teleopInit();
            } else if (mode == Mode.kTest) {
                testInit();
            }

            lastMode = mode;
        }

        // Call the appropriate function depending upon the current robot mode
        if (mode == Mode.kDisabled) {
            DriverStationJNI.observeUserProgramDisabled();
            Tracer.traceFunc("disabledPeriodic", this::disabledPeriodic);
        } else if (mode == Mode.kAutonomous) {
            DriverStationJNI.observeUserProgramAutonomous();
            Tracer.traceFunc("autonomousPeriodic", this::autonomousPeriodic);
        } else if (mode == Mode.kTeleop) {
            DriverStationJNI.observeUserProgramTeleop();
            Tracer.traceFunc("teleopPeriodic", this::teleopPeriodic);
        } else {
            DriverStationJNI.observeUserProgramTest();
            Tracer.traceFunc("testPeriodic", this::testPeriodic);
        }

        Tracer.traceFunc("robotPeriodic", this::robotPeriodic);

        SmartDashboard.updateValues();

        if (isSimulation()) {
            HAL.simPeriodicBefore();
            simulationPeriodic();
            HAL.simPeriodicAfter();
        }

        NetworkTableInstance.getDefault().flushLocal();
    }
}
