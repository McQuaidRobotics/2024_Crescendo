package com.igknighters.subsystems.stem;

import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kStem.kTelescope;

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
    private final MechanismLigament2d telescopeCurrent, wristLowerCurrent, wristUpperCurrent;
    private final MechanismLigament2d telescopeSetpoint, wristLowerSetpoint, wristUpperSetpoint;

    public StemVisualizer() {
        mechanism = new Mechanism2d(2.0, 2.0);

        rootCurrent = mechanism.getRoot("Current", 0.5, 1.0);
        rootSetpoint = mechanism.getRoot("Setpoint", 0.5, 1.0);

        telescopeCurrent = rootCurrent.append(new MechanismLigament2d("Telescope Current", kTelescope.MIN_METERS, 0.0));
        wristLowerCurrent = telescopeCurrent.append(new MechanismLigament2d("Wrist Lower Current", 0.3, -54.0));
        wristUpperCurrent = telescopeCurrent.append(new MechanismLigament2d("Wrist Upper Current", 0.03, 180.0 - 54.0));

        wristLowerCurrent.setColor(new Color8Bit(255, 255, 255));
        wristUpperCurrent.setColor(new Color8Bit(255, 255, 255));

        telescopeSetpoint = rootSetpoint.append(new MechanismLigament2d("Telescope Setpoint", kTelescope.MIN_METERS, 0.0));
        wristLowerSetpoint = telescopeSetpoint.append(new MechanismLigament2d("Wrist Lower Setpoint", 0.3, -54.0));
        wristUpperSetpoint = telescopeSetpoint.append(new MechanismLigament2d("Wrist Upper Setpoint", 0.03, 180.0 - 54.0));

        telescopeSetpoint.setColor(new Color8Bit(170, 180, 180));
        wristLowerSetpoint.setColor(new Color8Bit(170, 180, 180));
        wristUpperSetpoint.setColor(new Color8Bit(170, 180, 180));
        telescopeSetpoint.setLineWeight(3.0);
        wristLowerSetpoint.setLineWeight(3.0);
        wristUpperSetpoint.setLineWeight(3.0);

        wristLowerSetpoint.setColor(new Color8Bit(180, 180, 180));
        wristUpperSetpoint.setColor(new Color8Bit(180, 180, 180));

        var table = NetworkTableInstance.getDefault()
            .getTable("Visualizers")
            .getSubTable("Stem");
        var builder = new SendableBuilderImpl();
        builder.setTable(table);
        mechanism.initSendable(builder);
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

        wristLowerCurrent.setAngle(-Units.radiansToDegrees(currentPose.wristRads));
        wristUpperCurrent.setAngle(180.0 - Units.radiansToDegrees(currentPose.wristRads));
    }

    public void updateSetpoint(StemPosition desiredPose) {
        telescopeSetpoint.setAngle((Units.radiansToDegrees(desiredPose.pivotRads)));
        telescopeSetpoint.setLength(desiredPose.telescopeMeters);
        wristLowerSetpoint.setAngle(-Units.radiansToDegrees(desiredPose.wristRads));
        wristUpperSetpoint.setAngle(180.0 - Units.radiansToDegrees(desiredPose.wristRads));
    }
}
