package com.igknighters.commands.swerve;

import com.igknighters.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import com.igknighters.GlobalState;
import com.igknighters.commands.Helpers;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;

public class TeleopSwerveTarget extends Command {

    private final Swerve swerve;
    private final DoubleSupplier translationXSup;
    private final DoubleSupplier translationYSup;
    private final DoubleSupplier targetXSup;
    private final DoubleSupplier targetYSup;

    private Translation2d targetTranslation = new Translation2d(5.0, 5.0);

    public TeleopSwerveTarget(
            Swerve swerve,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier targetX,
            DoubleSupplier targetY) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationXSup = Helpers.deadbandSupplier(translationX, 0.1);
        this.translationYSup = Helpers.deadbandSupplier(translationY, 0.1);
        this.targetXSup = Helpers.deadbandSupplier(targetX, 0.1);
        this.targetYSup = Helpers.deadbandSupplier(targetY, 0.1);
    }

    @Override
    public void execute() {
        targetTranslation = targetTranslation
            .plus(new Translation2d(targetXSup.getAsDouble(), targetYSup.getAsDouble()));

        var vx = translationXSup.getAsDouble() * kSwerve.MAX_DRIVE_VELOCITY * 0.85;
        var vy = translationYSup.getAsDouble() * kSwerve.MAX_DRIVE_VELOCITY * 0.85;

        GlobalState.modifyField(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        var targetAngle = swerve.rotationRelativeToPose(
            new Rotation2d(),
            targetTranslation.plus(new Translation2d(
                vx * ConstValues.PERIODIC_TIME,
                vy * ConstValues.PERIODIC_TIME
            ))
        );
        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx,
            vy,
            rotVelo,
            swerve.getYawRot()
        );

        swerve.driveChassisSpeeds(chassisSpeeds, true, false);
    }
}
