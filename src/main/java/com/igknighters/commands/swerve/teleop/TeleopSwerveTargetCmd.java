package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.RotationalController;
import com.igknighters.subsystems.swerve.Swerve;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.geom.GeomUtil;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.util.function.Supplier;

import com.igknighters.Localizer;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.controllers.ControllerBase;

public class TeleopSwerveTargetCmd extends TeleopSwerveBaseCmd {
    private static final TunableDouble lookaheadTime = TunableValues.getDouble("SwerveTargetCmd/AutoAimLookaheadTime", 0.2);

    private final Supplier<Translation2d> translationSupplier;
    private final Translation2d target;
    private final boolean movementComp;
    private final RotationalController rotController;
    private final double speedMult;

    public TeleopSwerveTargetCmd(Swerve swerve, ControllerBase controller, Localizer localizer, Translation2d target, boolean movementComp, double speedScalar) {
        this(swerve, controller, localizer::translation, target, movementComp, speedScalar);
    }

    public TeleopSwerveTargetCmd(Swerve swerve, ControllerBase controller, Supplier<Translation2d> poseSupplier, Translation2d target, boolean movementComp, double speedScalar) {
        super(swerve, controller);
        addRequirements(swerve);
        this.translationSupplier = poseSupplier;
        this.target = target;
        this.movementComp = movementComp;
        this.rotController = new RotationalController(swerve);
        this.speedMult = speedScalar;
    }

    @Override
    public void initialize() {
        rotController.reset();
    }

    @Override
    public void execute() {
        Translation2d currentTranslation = translationSupplier.get();
        Translation2d targetTranslation = AllianceFlip.isBlue() ? target : AllianceFlip.flipTranslation(target);

        Translation2d vt = orientForUser(getTranslation())
                .times(kSwerve.MAX_DRIVE_VELOCITY * speedMult);

        ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vt.getX(),
                vt.getY(),
                0.0,
                new Rotation2d(swerve.getYawRads()));
        ChassisSpeeds currentChassisSpeeds = swerve.getChassisSpeed();

        ChassisSpeeds avgChassisSpeeds = new ChassisSpeeds(
                (desiredChassisSpeeds.vxMetersPerSecond + currentChassisSpeeds.vxMetersPerSecond) / 2.0,
                (desiredChassisSpeeds.vyMetersPerSecond + currentChassisSpeeds.vyMetersPerSecond) / 2.0,
                0.0);

        double distance = currentTranslation.getDistance(targetTranslation);

        double noteVelo = TunableValues.getDouble("Note Average Velo", kUmbrella.NOTE_VELO).value();

        Translation2d adjustedTarget = new Translation2d(
                targetTranslation.getX() - (avgChassisSpeeds.vxMetersPerSecond * (distance / noteVelo)),
                targetTranslation.getY() - (avgChassisSpeeds.vyMetersPerSecond * (distance / noteVelo)));

        double lookaheadTimeValue = lookaheadTime.value();
        Translation2d lookaheadTranslation = currentTranslation.minus(new Translation2d(
                avgChassisSpeeds.vxMetersPerSecond * lookaheadTimeValue,
                avgChassisSpeeds.vyMetersPerSecond * lookaheadTimeValue
            ));

        Rotation2d targetAngle;

        if (movementComp) {
            targetAngle = GeomUtil.rotationRelativeToPose(
                lookaheadTranslation,
                adjustedTarget
            ).plus(GeomUtil.ROTATION2D_PI);
        } else {
            targetAngle = GeomUtil.rotationRelativeToPose(
                currentTranslation,
                targetTranslation
            ).plus(GeomUtil.ROTATION2D_PI);
        }

        desiredChassisSpeeds.omegaRadiansPerSecond = rotController.calculate(targetAngle.getRadians(), Units.degreesToRadians(0.3));

        swerve.drive(desiredChassisSpeeds, false);
    }

    public static final TeleopSwerveBaseStruct struct = new TeleopSwerveBaseStruct();
}
