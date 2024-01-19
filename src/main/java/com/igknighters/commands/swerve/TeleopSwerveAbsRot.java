package com.igknighters.commands.swerve;

import com.igknighters.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    private final PIDController rotController = new PIDController(20.0, 0.0, 0.0);
    // private final TrapezoidProfile rotProfile = new TrapezoidProfile(
    // new Constraints(kSwerve.MAX_ANGULAR_VELOCITY, kSwerve.MAX_ANGULAR_VELOCITY *
    // 10.0)
    // );

    /**if within x of a multiple of 90 degrees it will snap to the closest multiple of 90 degrees */
    private final static double ANGLE_SNAP_WINDOW = Units.degreesToRadians(5.0);

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
        // the angle of the translation vector
        double wantedAngle = Math.atan2(absRotation.getY(), absRotation.getX());
        // a 0-1 value representing the magnitude of the translation vector
        double magnitude = absRotation.getNorm();
        // the current angle reading of the gyro
        double currentAngle = Units.degreesToRadians(swerve.getYaw());

        // Snap to the closest multiple of 90 degrees if within the snap window
        if (Math.abs(wantedAngle % (Math.PI / 2)) < ANGLE_SNAP_WINDOW) {
            wantedAngle = Math.round(wantedAngle / (Math.PI / 2)) * (Math.PI / 2);
        }

        // if our current angle has wrapped around, we need to adjust our wanted angle
        if (Math.abs(wantedAngle - currentAngle) > Math.PI) {
            if (wantedAngle > currentAngle) {
                wantedAngle -= 2 * Math.PI;
            } else {
                wantedAngle += 2 * Math.PI;
            }
        }

        SmartDashboard.putNumber("wantedAngle", wantedAngle);
        SmartDashboard.putNumber("currentAngle", currentAngle);

        var rotVelo = rotController.calculate(currentAngle, wantedAngle);

        // Make sure we take the shortest path to our goal
        // if (Math.abs(wantedAngle - currentAngle) > Math.PI) {
        //     rotVelo = -rotVelo;
        // }

        SmartDashboard.putNumber("rotVelo", rotVelo);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX() * kSwerve.MAX_DRIVE_VELOCITY,
            translation.getY() * kSwerve.MAX_DRIVE_VELOCITY,
            rotVelo * magnitude,
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
