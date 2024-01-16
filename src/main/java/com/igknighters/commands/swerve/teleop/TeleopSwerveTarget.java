package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.igknighters.GlobalState;
import com.igknighters.constants.ConstValues;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveTarget extends TeleopSwerveBase {

    private Translation2d targetTranslation = new Translation2d(5.0, 5.0);

    public TeleopSwerveTarget(Swerve swerve, ControllerParent controller) {
        super(swerve, controller);
    }

    @Override
    public void execute() {
            targetTranslation = targetTranslation
                .plus(
                    adjustForSimOrientedControl(
                        new Translation2d(
                            getRotationX() * 0.1,
                            getRotationY() * 0.1)
                        ));

        var vt = adjustForSimOrientedControl(new Translation2d(
            getTranslationX() * kSwerve.MAX_DRIVE_VELOCITY * 0.85,
            getTranslationY() * kSwerve.MAX_DRIVE_VELOCITY * 0.85));

        GlobalState.modifyField(field -> {
            field.getObject("target").setPose(new Pose2d(targetTranslation, new Rotation2d()));
        });

        var targetAngle = swerve.rotationRelativeToPose(
            new Rotation2d(),
            targetTranslation.plus(new Translation2d(
                vt.getX() * ConstValues.PERIODIC_TIME,
                vt.getY() * ConstValues.PERIODIC_TIME
            ))
        );
        var rotVelo = swerve.rotVeloForRotation(targetAngle);

        SmartDashboard.putNumber("vx", vt.getX());
        SmartDashboard.putNumber("vy", vt.getY());

        var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vt.getX(),
            vt.getY(),
            rotVelo,
            swerve.getYawRot()
        );

        swerve.driveChassisSpeeds(chassisSpeeds, true, false);
    }
}
