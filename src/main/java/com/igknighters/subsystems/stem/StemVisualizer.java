package com.igknighters.subsystems.stem;

import com.igknighters.constants.ConstValues.kRobotGeometry;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kTelescope;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // --- Temporary for debugging ---
    private MechanismRoot2d pivotAxelPointRoot;
    private MechanismRoot2d wristAxelPointRoot;
    private MechanismRoot2d umbrellaBottomLeftPointRoot;
    private MechanismRoot2d umbrellaBottomRightPointRoot;
    private MechanismRoot2d umbrellaTopLeftPointRoot;
    private MechanismRoot2d umbrellaTopRightPointRoot;

    private final double wristOffset = 90;

    public StemVisualizer() {
        mechanism = new Mechanism2d(2.0, 2.0);

        mechanism.setBackgroundColor(backgroundColor);

        rootCurrent = mechanism.getRoot("Current", pivotOrigin.getX(), pivotOrigin.getY());
        rootSetpoint = mechanism.getRoot("Setpoint", pivotOrigin.getX(), pivotOrigin.getY());

        telescopeCurrent = rootCurrent.append(new MechanismLigament2d("Telescope Current", kTelescope.MIN_METERS, 0.0));
        wristCurrent = telescopeCurrent
                .append(new MechanismLigament2d("Wrist Lower Current", kRobotGeometry.UMBRELLA_OFFSET,
                        -54.0 + wristOffset));

        wristCurrent.setColor(backgroundColor);
        wristCurrent.setLineWeight(4.0);

        telescopeSetpoint = rootSetpoint
                .append(new MechanismLigament2d("Telescope Setpoint", kTelescope.MIN_METERS, 0.0));
        wristSetpoint = telescopeSetpoint
                .append(new MechanismLigament2d("Wrist Lower Setpoint", kRobotGeometry.UMBRELLA_OFFSET,
                        -54.0 + wristOffset));

        telescopeSetpoint.setColor(debugColor);
        telescopeSetpoint.setLineWeight(3.0);

        wristSetpoint.setColor(backgroundColor);
        wristSetpoint.setLineWeight(1.0);

        drawDriveBase();
        drawUmbrella(wristCurrent, false);
        drawUmbrella(wristSetpoint, true);
        drawMaxBounds();

        pivotAxelPointRoot = mechanism.getRoot("pivotAxelPoint", 0, 0);
        wristAxelPointRoot = mechanism.getRoot("wristAxelPoint", 0, 0);
        umbrellaBottomLeftPointRoot = mechanism.getRoot("umbrellaBottomLeftPoint", 0, 0);
        umbrellaBottomRightPointRoot = mechanism.getRoot("umbrellaBottomRightPoint", 0, 0);
        umbrellaTopLeftPointRoot = mechanism.getRoot("umbrellaTopLeftPoint", 0, 0);
        umbrellaTopRightPointRoot = mechanism.getRoot("umbrellaTopRightPoint", 0, 0);

        pivotAxelPointRoot.append(
                new MechanismLigament2d(
                        pivotAxelPointRoot.getName(), 
                        0.02, 
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );
        wristAxelPointRoot.append(
                new MechanismLigament2d(
                        wristAxelPointRoot.getName(), 
                        0.02,
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );
        umbrellaBottomLeftPointRoot.append(
                new MechanismLigament2d(
                        umbrellaBottomLeftPointRoot.getName(), 
                        0.02,
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );
        umbrellaBottomRightPointRoot.append(
                new MechanismLigament2d(
                        umbrellaBottomRightPointRoot.getName(), 
                        0.02,
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );
        umbrellaTopLeftPointRoot.append(
                new MechanismLigament2d(
                        umbrellaTopLeftPointRoot.getName(), 
                        0.02,
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );
        umbrellaTopRightPointRoot.append(
                new MechanismLigament2d(
                        umbrellaTopRightPointRoot.getName(), 
                        0.02, 
                        0.0,
                        2.0,
                        new Color8Bit(255, 255, 255))
        );

        var table = NetworkTableInstance.getDefault()
                .getTable("Visualizers")
                .getSubTable("Stem");
        var builder = new SendableBuilderImpl();
        builder.setTable(table);
        mechanism.initSendable(builder);
    }

    private void drawUmbrella(MechanismLigament2d wristRoot, boolean isSetpoint) {
        MechanismLigament2d leftSide = wristRoot.append(
                new MechanismLigament2d(
                        "Umbrella Left Side",
                        kRobotGeometry.UMBRELLA_HEIGHT,
                        0.0));
        MechanismLigament2d topSide = leftSide.append(
                new MechanismLigament2d(
                        "Umbrella Top Side",
                        kRobotGeometry.UMBRELLA_LENGTH,
                        -90.0));
        MechanismLigament2d rightSide = topSide.append(
                new MechanismLigament2d(
                        "Umbrella Right Side",
                        kRobotGeometry.UMBRELLA_HEIGHT,
                        -90.0));
        MechanismLigament2d bottomSide = rightSide.append(
                new MechanismLigament2d(
                        "Umbrella Bottom Side",
                        kRobotGeometry.UMBRELLA_LENGTH,
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
        Translation2d drivebaseOrigin = pivotOrigin.minus(kRobotGeometry.PIVOT_LOCATION);

        MechanismRoot2d drivebaseRoot = mechanism.getRoot("Drivebase", drivebaseOrigin.getX(), drivebaseOrigin.getY());
        MechanismLigament2d leftSide = drivebaseRoot.append(
                new MechanismLigament2d(
                        "DriveBase Left Side",
                        kRobotGeometry.DRIVE_BASE.getHeight(),
                        90.0));
        MechanismLigament2d topSide = leftSide.append(
                new MechanismLigament2d(
                        "DriveBase Top Side",
                        kRobotGeometry.DRIVE_BASE.getWidth(),
                        -90.0));
        MechanismLigament2d rightSide = topSide.append(
                new MechanismLigament2d(
                        "DriveBase Right Side",
                        kRobotGeometry.DRIVE_BASE.getHeight(),
                        -90.0));
        MechanismLigament2d bottomSide = rightSide.append(
                new MechanismLigament2d(
                        "DriveBase Bottom Side",
                        kRobotGeometry.DRIVE_BASE.getWidth(),
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
        Translation2d drivebaseOrigin = pivotOrigin.minus(kRobotGeometry.PIVOT_LOCATION);

        MechanismRoot2d boundsRoot = mechanism.getRoot("MaxBounds", drivebaseOrigin.getX(), drivebaseOrigin.getY());
        MechanismLigament2d leftBottomSide = boundsRoot.append(
                new MechanismLigament2d(
                        "MaxBounds Left Bottom Side",
                        Units.inchesToMeters(12.0) - ((kRobotGeometry.DRIVE_BASE.getWidth() - Units.inchesToMeters(26.0)) / 2.0),
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
                        Units.inchesToMeters(12.0) - ((kRobotGeometry.DRIVE_BASE.getWidth() - Units.inchesToMeters(26.0)) / 2.0),
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

        SmartDashboard.putString("Stem Position Valid", StemValidator.isValidPosition(currentPose).name());

        Translation2d drivebaseOrigin = pivotOrigin.minus(kRobotGeometry.PIVOT_LOCATION);
        StemValidator.MechanismPoints mechPts = new StemValidator.MechanismPoints(currentPose);
        var pivotAxelPoint = kRobotGeometry.PIVOT_LOCATION.plus(drivebaseOrigin);
        var wristAxelPoint = mechPts.wristAxelPoint.plus(drivebaseOrigin);
        var umbrellaBottomLeftPoint = mechPts.umbrellaBottomLeftPoint.plus(drivebaseOrigin);
        var umbrellaBottomRightPoint = mechPts.umbrellaBottomRightPoint.plus(drivebaseOrigin);
        var umbrellaTopLeftPoint = mechPts.umbrellaTopLeftPoint.plus(drivebaseOrigin);
        var umbrellaTopRightPoint = mechPts.umbrellaTopRightPoint.plus(drivebaseOrigin);

        pivotAxelPointRoot.setPosition(pivotAxelPoint.getX(), pivotAxelPoint.getY());
        wristAxelPointRoot.setPosition(wristAxelPoint.getX(), wristAxelPoint.getY());
        umbrellaBottomLeftPointRoot.setPosition(umbrellaBottomLeftPoint.getX(), umbrellaBottomLeftPoint.getY());
        umbrellaBottomRightPointRoot.setPosition(umbrellaBottomRightPoint.getX(), umbrellaBottomRightPoint.getY()); 
        umbrellaTopLeftPointRoot.setPosition(umbrellaTopLeftPoint.getX(), umbrellaTopLeftPoint.getY());
        umbrellaTopRightPointRoot.setPosition(umbrellaTopRightPoint.getX(), umbrellaTopRightPoint.getY());

        SmartDashboard.putString("pivotAxelPoint", kRobotGeometry.PIVOT_LOCATION.toString());
        SmartDashboard.putString("wristAxelPoint", mechPts.wristAxelPoint.toString());
        SmartDashboard.putString("umbrellaBottomLeftPoint", mechPts.umbrellaBottomLeftPoint.toString());
        SmartDashboard.putString("umbrellaBottomRightPoint", mechPts.umbrellaBottomRightPoint.toString());
        SmartDashboard.putString("umbrellaTopLeftPoint", mechPts.umbrellaTopLeftPoint.toString());
        SmartDashboard.putString("umbrellaTopRightPoint", mechPts.umbrellaTopRightPoint.toString());

        SmartDashboard.putString("Drive Base Bottom Left", kRobotGeometry.DRIVE_BASE.getBottomLeft().toString());
        SmartDashboard.putString("Drive Base Bottom Right", kRobotGeometry.DRIVE_BASE.getBottomRight().toString());
        SmartDashboard.putString("Drive Base Top Left", kRobotGeometry.DRIVE_BASE.getTopLeft().toString());
        SmartDashboard.putString("Drive Base Top Right", kRobotGeometry.DRIVE_BASE.getTopRight().toString());

        SmartDashboard.putString("Allowed Bounds Bottom Left", kRobotGeometry.BOUNDS.getBottomLeft().toString());
        SmartDashboard.putString("Allowed Bounds Bottom Right", kRobotGeometry.BOUNDS.getBottomRight().toString());
        SmartDashboard.putString("Allowed Bounds Top Left", kRobotGeometry.BOUNDS.getTopLeft().toString());
        SmartDashboard.putString("Allowed Bounds Top Right", kRobotGeometry.BOUNDS.getTopRight().toString());
    }

    public void updateSetpoint(StemPosition desiredPose) {
        telescopeSetpoint.setAngle((Units.radiansToDegrees(desiredPose.pivotRads)));
        telescopeSetpoint.setLength(desiredPose.telescopeMeters);
        wristSetpoint.setAngle(-Units.radiansToDegrees(desiredPose.wristRads) + wristOffset);
    }
}
