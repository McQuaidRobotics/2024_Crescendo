package com.igknighters.commands.swerve;

import com.igknighters.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.igknighters.constants.ConstValues.kSwerve;

public class TeleopSwerveSim extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Swerve swerve;
    private final DoubleSupplier translationXSup;
    private final DoubleSupplier translationYSup;
    private final DoubleSupplier rotationAxisSup;

    public TeleopSwerveSim(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationXSup = translation;
        this.translationYSup = strafe;
        this.rotationAxisSup = rotation;
    }

    @Override
    public void execute() {
        double translationVal;
        double strafeVal;
        double rotationVal;

        translationVal = MathUtil.applyDeadband(-translationXSup.getAsDouble(), 0.1);
        strafeVal = MathUtil.applyDeadband(-translationYSup.getAsDouble(), 0.1);
        rotationVal = MathUtil.applyDeadband(rotationAxisSup.getAsDouble(), 0.1);

        swerve.drive(
                new Translation2d(-strafeVal, translationVal)
                        .times(kSwerve.MAX_DRIVE_VELOCITY),
                rotationVal * kSwerve.MAX_ANGULAR_VELOCITY,
                true,
                true);
    }
}
