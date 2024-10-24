package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import java.nio.ByteBuffer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import com.igknighters.Robot;
import edu.wpi.first.wpilibj2.command.Command;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerBase;

public class TeleopSwerveBaseCmd extends Command implements StructSerializable {
    private static boolean shouldOrientForSim() {
        return Robot.isSimulation()
            && TunableValues.getBoolean("OrientTeleopForSim", kSwerve.ORIENT_TELEOP_FOR_SIM_DEFAULT).value();
    }

    /**
     * Orient the chassis speeds for the user,
     * If real robot this means up on translation stick moves away from driver.
     * If simulation and OrientTeleopForSim is true,
     * it will make up on the translation stick move up on the field visualization.
     * If simulation and OrientTeleopForSim is false it will
     * replicate the real robot.
     * 
     * @param input The controller input
     * @return The adjusted controller input
     */
    public static Translation2d orientForUser(Translation2d input) {
        if (shouldOrientForSim()) {
            return new Translation2d(
                    input.getX(),
                    -input.getY());
        } else {
            Translation2d chassisSpeedsAdj = input.rotateBy(Rotation2d.fromDegrees(-90));
            return new Translation2d(-chassisSpeedsAdj.getX(), chassisSpeedsAdj.getY());
        }
    }

    protected final Swerve swerve;

    private final DoubleSupplier rawTranslationXSup;
    private final DoubleSupplier rawTranslationYSup;
    private final DoubleSupplier rawRotationXSup;
    private final DoubleSupplier rawRotationYSup;

    private final TunableDouble translationMod;
    private final TunableDouble rotationMod;

    public TeleopSwerveBaseCmd(Swerve swerve, ControllerBase controller) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.rawTranslationXSup = controller.leftStickX();
        this.rawTranslationYSup = controller.leftStickY();
        this.rawRotationXSup = controller.rightStickX();
        this.rawRotationYSup = controller.rightStickY();

        if (Robot.isDemo()) {
            translationMod = TunableValues.getDouble("DemoSwerveTranslationModifier", 0.8);
            rotationMod = TunableValues.getDouble("DemoSwerveRotationalModifier", 0.8);
        } else {
            translationMod = null;
            rotationMod = null;
        }
    }

    private double invert() {
        if (shouldOrientForSim()) {
            return 1;
        } else if (AllianceFlip.isRed()) {
            return -1;
        } else {
            return 1;
        }
    }

    protected Translation2d getTranslation() {
        double rawX = -rawTranslationXSup.getAsDouble();
        double rawY = rawTranslationYSup.getAsDouble();
        double angle = Math.atan2(rawY, rawX);
        double rawMagnitude = Math.hypot(rawX, rawY);
        double magnitude = kSwerve.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
        if (Robot.isDemo()) magnitude *= translationMod.value();
        double processedX = magnitude * Math.cos(angle) * invert();
        double processedY = magnitude * Math.sin(angle) * invert();
        return new Translation2d(processedX, processedY);
    }

    protected Translation2d getRotation() {
        double rawX = -rawRotationXSup.getAsDouble();
        double rawY = rawRotationYSup.getAsDouble();
        double angle = Math.atan2(rawY, rawX);
        double rawMagnitude = Math.hypot(rawX, rawY);
        double magnitude = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawMagnitude);
        if (Robot.isDemo()) magnitude *= rotationMod.value();
        double processedX = magnitude * Math.cos(angle);
        double processedY = magnitude * Math.sin(angle);
        return new Translation2d(processedX, processedY);
    }

    protected double getRotationX() {
        double rawX = -rawRotationXSup.getAsDouble();
        double processed = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawX);
        if (Robot.isDemo()) processed *= rotationMod.value();
        return processed;
    }

    public static class TeleopSwerveBaseStruct implements Struct<TeleopSwerveBaseCmd> {
        @Override
        public Class<TeleopSwerveBaseCmd> getTypeClass() {
            return TeleopSwerveBaseCmd.class;
        }

        @Override
        public int getSize() {
            return 8*8;
        }

        @Override
        public String getSchema() {
            return "double RawTranslationX; "
                + "double TranslationX; "
                + "double RawTranslationY; "
                + "double TranslationY; "
                + "double RawRotationX; "
                + "double RotationX; "
                + "double RawRotationY; "
                + "double RotationY;";
        }

        @Override
        public String getTypeString() {
            return "struct:" + getTypeClass().getSimpleName();
        }

        @Override
        public void pack(ByteBuffer bb, TeleopSwerveBaseCmd value) {
            if (value.isScheduled()) {
                Translation2d translation = value.getTranslation();
                Translation2d rotation = value.getRotation();
                bb.putDouble(value.rawTranslationXSup.getAsDouble());
                bb.putDouble(translation.getX());
                bb.putDouble(value.rawTranslationYSup.getAsDouble());
                bb.putDouble(translation.getY());
                bb.putDouble(value.rawRotationXSup.getAsDouble());
                bb.putDouble(rotation.getX());
                bb.putDouble(value.rawRotationYSup.getAsDouble());
                bb.putDouble(rotation.getY());
            } else {
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
                bb.putDouble(0.0);
            }
        }

        @Override
        public TeleopSwerveBaseCmd unpack(ByteBuffer bb) {
            throw new UnsupportedOperationException();
        }
    }

    public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
