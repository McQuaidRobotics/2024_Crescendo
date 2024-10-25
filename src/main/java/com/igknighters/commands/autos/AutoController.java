package com.igknighters.commands.autos;

import java.util.Optional;
import java.util.function.BiConsumer;

import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.subsystems.swerve.Swerve;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoController implements BiConsumer<Pose2d, SwerveSample> {
    private final Swerve swerve;
    private final boolean enabled;
    private final PIDController xController = new PIDController(
        kAuto.kTranslation.kP,
        kAuto.kTranslation.kI,
        kAuto.kTranslation.kD
    );
    private final PIDController yController = new PIDController(
        kAuto.kTranslation.kP,
        kAuto.kTranslation.kI,
        kAuto.kTranslation.kD
    );
    private final PIDController rController = new PIDController(
        kAuto.kRotation.kP,
        kAuto.kRotation.kI,
        kAuto.kRotation.kD
    );

    public AutoController(Optional<Swerve> swerve) {
        if (swerve.isEmpty()) {
            this.swerve = null;
            this.enabled = false;
            return;
        }
        this.swerve = swerve.get();
        this.enabled = true;
        rController.enableContinuousInput(-Math.PI, Math.PI);
        xController.close();
        yController.close();
        rController.close();
    }

    @Override
    public void accept(Pose2d pose, SwerveSample referenceState) {
        if (!enabled) {
            return;
        }
        double xFF = referenceState.vx;
        double yFF = referenceState.vy;
        double rotationFF = referenceState.omega;

        double xFeedback = xController.calculate(pose.getX(), referenceState.x);
        double yFeedback = yController.calculate(pose.getY(), referenceState.y);
        double rotationFeedback = rController.calculate(pose.getRotation().getRadians(),
            referenceState.heading);

        ChassisSpeeds out = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            pose.getRotation()
        );

        swerve.drive(out, false);
    }
}
