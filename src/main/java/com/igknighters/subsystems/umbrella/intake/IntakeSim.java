package com.igknighters.subsystems.umbrella.intake;

import java.util.ArrayList;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues.kRobotCollisionGeometry;
import com.igknighters.constants.ConstValues.kUmbrella.kIntake;
import com.igknighters.util.BootupLogger;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.Rectangle2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakeSim extends Intake {

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

    private final ArrayList<Rectangle2d> autoNotes = new ArrayList<>();
    private boolean isAuto = false;

    public IntakeSim() {
        BootupLogger.bootupLog("    Intake initialized (sim)");
    }

    private void setExitBeam(boolean broken) {
        super.exitBeamBroken = broken;
    }

    @Override
    public void setVoltageOut(double volts) {
        setVoltageOut(volts, false);
    }

    @Override
    public boolean isExitBeamBroken() {
        return super.exitBeamBroken || (DriverStation.isAutonomousEnabled() && !isAuto);
    }

    @Override
    public void setVoltageOut(double volts, boolean force) {
        if (isExitBeamBroken() && !force) {
            volts = 0.0;
        }

        if (force || volts > 0.0) {
            setExitBeam(false);
        }

        super.voltsLower = volts;
        super.voltsUpper = super.voltsLower * kIntake.UPPER_DIFF;
        super.radiansPerSecondLower = (volts / 12.0) * DCMotor.getFalcon500(0).freeSpeedRadPerSec;
        super.radiansPerSecondUpper = super.radiansPerSecondLower * kIntake.UPPER_DIFF;
    }

    private boolean isIntaking() {
        return super.voltsLower < 0.0 && super.voltsUpper < 0.0;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) setVoltageOut(0.0);

        boolean nowAuto = DriverStation.isAutonomousEnabled();

        if (nowAuto && !isAuto) {
            autoNotes.clear();
            for (var note : AutoNotes.notes) {
            autoNotes.add(new Rectangle2d(
                    AllianceFlip.isBlue() ? note : AllianceFlip.flipTranslation(note),
                    Units.inchesToMeters(7.0)));
            }
            setExitBeam(true);
        }

        if (isAuto && nowAuto) {
            Translation2d intakePose = GlobalState.getLocalizedPose().transformBy(AutoNotes.intakeTransform).getTranslation();
            ArrayList<Integer> notesToRemove = new ArrayList<>();
            for (int i = 0; i < autoNotes.size(); i++) {
                if (autoNotes.get(i).contains(intakePose) && isIntaking()) {
                    notesToRemove.add(i);
                    System.out.println("Intaked note at " + autoNotes.get(i).getCenter());
                    setExitBeam(true);
                }
            }
            for (int i : notesToRemove) {
                autoNotes.remove(i);
            }

            GlobalState.modifyField2d(field -> {
                Pose2d[] notePoses = this.autoNotes.stream()
                        .map(Rectangle2d::getCenter)
                        .map(t2d -> new Pose2d(t2d, new Rotation2d()))
                        .toArray(Pose2d[]::new);

                field.getObject("AutoNotes").setPoses(notePoses);

                if (isIntaking()) {
                    field.getObject("Intake").setPose(new Pose2d(intakePose, new Rotation2d()));
                } else {
                    field.getObject("Intake").setPoses();
                }
            });
        }

        if (!nowAuto && isAuto) {
            autoNotes.clear();
            GlobalState.modifyField2d(field -> {
                field.getObject("AutoNotes").setPoses();
                field.getObject("Intake").setPoses();
            });
        }

        isAuto = nowAuto;
    }
}
