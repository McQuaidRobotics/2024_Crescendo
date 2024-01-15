package com.igknighters.commands.swerve;

import com.igknighters.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.igknighters.commands.Helpers;
import com.igknighters.constants.ConstValues.kSwerve;

/** An example command that uses an example subsystem. */
public class TeleopSwerveAbsRot extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Swerve swerve;
    private final DoubleSupplier translationXSup;
    private final DoubleSupplier translationYSup;
    private final DoubleSupplier rotationXSup;
    private final DoubleSupplier rotationYSup;

    public TeleopSwerveAbsRot(
            Swerve swerve,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotationX,
            DoubleSupplier rotationY) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationXSup = Helpers.deadbandSupplier(translationX, 0.1);
        this.translationYSup = Helpers.deadbandSupplier(translationY, 0.1);
        this.rotationXSup = Helpers.deadbandSupplier(rotationX, 0.1);
        this.rotationYSup = Helpers.deadbandSupplier(rotationY, 0.1);
    }

    public ChassisSpeeds genChassisSpeeds(Translation2d translation, Translation2d absRotation, boolean isOpenLoop) {
        var rotVelo = swerve.rotVeloForRotation(absRotation.getAngle());

        SmartDashboard.putNumber("rotVelo", rotVelo);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * kSwerve.MAX_DRIVE_VELOCITY,
            translation.getY() * kSwerve.MAX_DRIVE_VELOCITY,
            rotVelo * absRotation.getNorm(),
            swerve.getYawRot()
        );
    }

    @Override
    public void execute() {
        var chassisSpeeds = genChassisSpeeds(
                new Translation2d(
                        -translationYSup.getAsDouble(), -translationXSup.getAsDouble()),
                new Translation2d(
                        rotationXSup.getAsDouble(), rotationYSup.getAsDouble()),
                true);
        swerve.driveChassisSpeeds(chassisSpeeds, true, false);
    }
}
