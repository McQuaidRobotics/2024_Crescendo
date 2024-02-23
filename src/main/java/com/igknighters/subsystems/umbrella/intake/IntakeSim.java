package com.igknighters.subsystems.umbrella.intake;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kRobotCollisionGeometry;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.geom.Rectangle2d;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeSim implements Intake {

    private static final class AutoNotes {
        private static final Translation2d[] notes = {
                new Translation2d(2.9, 7.0),
                new Translation2d(2.9, 5.54),
                new Translation2d(2.9, 4.1),
                new Translation2d(8.3, 7.44),
                new Translation2d(8.3, 5.77),
                new Translation2d(8.3, 4.11),
                new Translation2d(8.3, 2.42),
                new Translation2d(8.3, 0.76),
        };

        private static final Transform2d intakeTransform = new Transform2d(
            new Translation2d(
                (kRobotCollisionGeometry.FRAME_WIDTH / 2.0)
                + kRobotCollisionGeometry.BUMPER_THICKNESS
                + kRobotCollisionGeometry.UMBRELLA_HEIGHT,
                0
            ),
            new Rotation2d()
        );
    }

    private final IntakeInputs inputs = new IntakeInputs();
    private final SimBoolean exit;

    private final ArrayList<Rectangle2d> autoNotes = new ArrayList<>();
    private boolean isAuto = false;

    public IntakeSim() {
        if (RobotBase.isReal()) {
            // In the event we use this to "disable" the intake on the real robot,
            // we don't want to crash the robot by trying to create a SimDevice
            exit = null;
            NetworkTableInstance.getDefault()
                    .getTable("SimDevices")
                    .getSubTable("ExitBeamBreak")
                    .getEntry("broken2")
                    .setBoolean(false);
        } else if (GlobalState.isUnitTest()) {
            // HAL requires unique allocations for each SimDevice,
            // in unit tests we don't care what this is actually called so just make it
            // random
            exit = SimDevice.create("" + Math.random() + Math.random()).createBoolean("", Direction.kInput,
                    false);
        } else {
            exit = SimDevice.create("ExitBeamBreak").createBoolean("broken2", Direction.kInput, false);
        }

        BootupLogger.bootupLog("    Intake initialized (sim)");
    }

    @Override
    public void setVoltageOut(double volts) {
        setVoltageOut(volts, false);
    }

    @Override
    public boolean isExitBeamBroken() {
        return inputs.exitBeamBroken;
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (isExitBeamBroken() && !force) {
            volts = 0.0;
        }
        inputs.voltsLower = volts;
        inputs.voltsUpper = inputs.voltsLower * kIntake.UPPER_DIFF;
        inputs.radiansPerSecondLower = (volts / 12.0) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
        inputs.radiansPerSecondUpper = inputs.radiansPerSecondLower * kIntake.UPPER_DIFF;

        if (force) {
            exit.set(false);
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            var table = NetworkTableInstance.getDefault()
                    .getTable("SimDevices");
            inputs.exitBeamBroken = table.getSubTable("ExitBeamBreak")
                    .getEntry("broken2")
                    .getBoolean(false);
        } else {
            inputs.exitBeamBroken = exit.get();
        }

        boolean nowAuto = DriverStation.isAutonomousEnabled();

        if (nowAuto && !isAuto) {
            autoNotes.clear();
            for (var note : AutoNotes.notes) {
                autoNotes.add(new Rectangle2d(
                    note,
                    Units.inchesToMeters(7.0)));
            }
        }

        if (isAuto && nowAuto) {
            Translation2d intakePose = GlobalState.getLocalizedPose().transformBy(AutoNotes.intakeTransform).getTranslation();
            ArrayList<Integer> notesToRemove = new ArrayList<>();
            for (int i = 0; i < autoNotes.size(); i++) {
                if (autoNotes.get(i).contains(intakePose) && inputs.voltsLower > 0.0) {
                    notesToRemove.add(i);
                    System.out.println("Intaked note at " + autoNotes.get(i).getBottomLeft());
                    if (RobotBase.isReal()) {
                        NetworkTableInstance.getDefault()
                                .getTable("SimDevices")
                                .getSubTable("ExitBeamBreak")
                                .getEntry("broken2")
                                .setBoolean(true);
                    } else {
                        exit.set(true);
                    }
                    inputs.exitBeamBroken = true;
                }
            }
            for (int i : notesToRemove) {
                autoNotes.remove(i);
            }
        }

        if (!nowAuto && isAuto) {
            autoNotes.clear();
        }

        isAuto = nowAuto;

        Logger.processInputs("/Umbrella/Intake", inputs);
    }
}
