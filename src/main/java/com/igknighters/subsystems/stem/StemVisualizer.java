package com.igknighters.subsystems.stem;

import com.igknighters.Robot;
import com.igknighters.constants.ConstValues.kRobotCollisionGeometry;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kTelescope;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class StemVisualizer {

        private final Mechanism2d mechanism;
        private final MechanismRoot2d rootCurrent, rootSetpoint;
        private final MechanismLigament2d telescopeCurrent, wristCurrent;
        private final MechanismLigament2d telescopeSetpoint, wristSetpoint;

        private final Color8Bit backgroundColor = new Color8Bit(0, 0, 0);
        private final Color8Bit debugColor = new Color8Bit(170, 180, 180);
        private final Color8Bit boundsColor = new Color8Bit(255, 180, 20);
        private final Translation2d pivotOrigin = new Translation2d(0.5, 0.5);

        private final double wristOffset = 90;

        public StemVisualizer() {
                mechanism = new Mechanism2d(2.0, 2.0);

                mechanism.setBackgroundColor(backgroundColor);

                rootCurrent = mechanism.getRoot("Current", pivotOrigin.getX(), pivotOrigin.getY());
                rootSetpoint = mechanism.getRoot("Setpoint", pivotOrigin.getX(), pivotOrigin.getY());

                telescopeCurrent = rootCurrent
                                .append(new MechanismLigament2d("Telescope Current", kTelescope.MIN_METERS, 0.0));
                wristCurrent = telescopeCurrent
                                .append(new MechanismLigament2d("Wrist Lower Current",
                                                kRobotCollisionGeometry.UMBRELLA_OFFSET,
                                                -54.0 + wristOffset));

                wristCurrent.setColor(backgroundColor);
                wristCurrent.setLineWeight(4.0);

                telescopeSetpoint = rootSetpoint
                                .append(new MechanismLigament2d("Telescope Setpoint", kTelescope.MIN_METERS, 0.0));
                wristSetpoint = telescopeSetpoint
                                .append(new MechanismLigament2d("Wrist Lower Setpoint",
                                                kRobotCollisionGeometry.UMBRELLA_OFFSET,
                                                -54.0 + wristOffset));

                telescopeSetpoint.setColor(debugColor);
                telescopeSetpoint.setLineWeight(3.0);

                wristSetpoint.setColor(backgroundColor);
                wristSetpoint.setLineWeight(1.0);

                drawDriveBase();
                drawUmbrella(wristCurrent, false);
                drawUmbrella(wristSetpoint, true);
                drawMaxBounds();

                if (Robot.isDebug()) {
                    var table = NetworkTableInstance.getDefault()
                        .getTable("Visualizers")
                        .getSubTable("Stem");
                    var builder = new SendableBuilderImpl();
                    builder.setTable(table);
                    mechanism.initSendable(builder);
                }
        }

        private void drawUmbrella(MechanismLigament2d wristRoot, boolean isSetpoint) {
                MechanismLigament2d leftSide = wristRoot.append(
                                new MechanismLigament2d(
                                                "Umbrella Left Side",
                                                kRobotCollisionGeometry.UMBRELLA_HEIGHT,
                                                0.0));
                MechanismLigament2d topSide = leftSide.append(
                                new MechanismLigament2d(
                                                "Umbrella Top Side",
                                                kRobotCollisionGeometry.UMBRELLA_LENGTH,
                                                -90.0));
                MechanismLigament2d rightSide = topSide.append(
                                new MechanismLigament2d(
                                                "Umbrella Right Side",
                                                kRobotCollisionGeometry.UMBRELLA_HEIGHT,
                                                -90.0));
                MechanismLigament2d bottomSide = rightSide.append(
                                new MechanismLigament2d(
                                                "Umbrella Bottom Side",
                                                kRobotCollisionGeometry.UMBRELLA_LENGTH,
                                                -90.0));

                if (isSetpoint) {
                        Color8Bit wristColor = new Color8Bit(200, 200, 255);
                        leftSide.setColor(wristColor);
                        topSide.setColor(wristColor);
                        rightSide.setColor(wristColor);
                        bottomSide.setColor(wristColor);

                        leftSide.setLineWeight(4.0);
                        topSide.setLineWeight(4.0);
                        rightSide.setLineWeight(4.0);
                        bottomSide.setLineWeight(4.0);
                } else {
                        leftSide.setColor(debugColor);
                        topSide.setColor(debugColor);
                        rightSide.setColor(debugColor);
                        bottomSide.setColor(debugColor);

                        leftSide.setLineWeight(3.0);
                        topSide.setLineWeight(3.0);
                        rightSide.setLineWeight(3.0);
                        bottomSide.setLineWeight(3.0);
                }
        }

        private void drawDriveBase() {
                Translation2d drivebaseOrigin = pivotOrigin.minus(kRobotCollisionGeometry.PIVOT_LOCATION);

                MechanismRoot2d drivebaseRoot = mechanism.getRoot("Drivebase", drivebaseOrigin.getX(),
                                drivebaseOrigin.getY());
                MechanismLigament2d leftSide = drivebaseRoot.append(
                                new MechanismLigament2d(
                                                "DriveBase Left Side",
                                                kRobotCollisionGeometry.DRIVE_BASE.getHeight(),
                                                90.0));
                MechanismLigament2d topSide = leftSide.append(
                                new MechanismLigament2d(
                                                "DriveBase Top Side",
                                                kRobotCollisionGeometry.DRIVE_BASE.getWidth(),
                                                -90.0));
                MechanismLigament2d rightSide = topSide.append(
                                new MechanismLigament2d(
                                                "DriveBase Right Side",
                                                kRobotCollisionGeometry.DRIVE_BASE.getHeight(),
                                                -90.0));
                MechanismLigament2d bottomSide = rightSide.append(
                                new MechanismLigament2d(
                                                "DriveBase Bottom Side",
                                                kRobotCollisionGeometry.DRIVE_BASE.getWidth(),
                                                -90.0));

                leftSide.setColor(new Color8Bit(100, 100, 100));
                topSide.setColor(new Color8Bit(100, 100, 100));
                rightSide.setColor(new Color8Bit(100, 100, 100));
                bottomSide.setColor(new Color8Bit(100, 100, 100));

                leftSide.setLineWeight(2.0);
                topSide.setLineWeight(2.0);
                rightSide.setLineWeight(2.0);
                bottomSide.setLineWeight(2.0);
        }

        private void drawMaxBounds() {
                Translation2d drivebaseOrigin = pivotOrigin.minus(kRobotCollisionGeometry.PIVOT_LOCATION);

                MechanismRoot2d boundsRoot = mechanism.getRoot("MaxBounds", drivebaseOrigin.getX(),
                                drivebaseOrigin.getY());
                MechanismLigament2d leftBottomSide = boundsRoot.append(
                                new MechanismLigament2d(
                                                "MaxBounds Left Bottom Side",
                                                Units.inchesToMeters(12.0)
                                                                - ((kRobotCollisionGeometry.DRIVE_BASE.getWidth()
                                                                                - Units.inchesToMeters(26.0)) / 2.0),
                                                180.0));
                MechanismLigament2d leftSide = leftBottomSide.append(
                                new MechanismLigament2d(
                                                "MaxBounds Left Side",
                                                Units.inchesToMeters(48.0),
                                                -90.0));
                MechanismLigament2d topSide = leftSide.append(
                                new MechanismLigament2d(
                                                "MaxBounds Top Side",
                                                Units.inchesToMeters(26.0 + 24.0),
                                                -90.0));
                MechanismLigament2d rightSide = topSide.append(
                                new MechanismLigament2d(
                                                "MaxBounds Right Side",
                                                Units.inchesToMeters(48.0),
                                                -90.0));
                MechanismLigament2d rightBottomSide = rightSide.append(
                                new MechanismLigament2d(
                                                "MaxBounds Right Bottom Side",
                                                Units.inchesToMeters(12.0)
                                                                - ((kRobotCollisionGeometry.DRIVE_BASE.getWidth()
                                                                                - Units.inchesToMeters(26.0)) / 2.0),
                                                -90.0));

                leftBottomSide.setColor(boundsColor);
                leftSide.setColor(boundsColor);
                topSide.setColor(boundsColor);
                rightSide.setColor(boundsColor);
                rightBottomSide.setColor(boundsColor);

                leftBottomSide.setLineWeight(2.0);
                leftSide.setLineWeight(2.0);
                topSide.setLineWeight(2.0);
                rightSide.setLineWeight(2.0);
                rightBottomSide.setLineWeight(2.0);
        }

        public void updateCurrent(StemPosition currentPose) {
                telescopeCurrent.setAngle(Units.radiansToDegrees(currentPose.pivotRads));
                telescopeCurrent.setLength(currentPose.telescopeMeters);

                // lerp the elevator color based on % of range
                // 0% = green, 100% = red
                Double percent = (currentPose.telescopeMeters - kStem.kTelescope.MIN_METERS)
                                / (kStem.kTelescope.MAX_METERS - kStem.kTelescope.MIN_METERS);
                int red = (int) (percent * 255);
                int green = (int) ((1 - percent) * 255);
                telescopeCurrent.setColor(new Color8Bit(red, green, 0));

                wristCurrent.setAngle(-Units.radiansToDegrees(currentPose.wristRads) + wristOffset);
        }

        public void updateSetpoint(StemPosition desiredPose) {
                telescopeSetpoint.setAngle((Units.radiansToDegrees(desiredPose.pivotRads)));
                telescopeSetpoint.setLength(desiredPose.telescopeMeters);
                wristSetpoint.setAngle(-Units.radiansToDegrees(desiredPose.wristRads) + wristOffset);
        }
}
