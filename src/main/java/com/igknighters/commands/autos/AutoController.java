package com.igknighters.commands.autos;

import com.igknighters.constants.ConstValues.kAuto;

import choreo.Choreo.ControlFunction;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoController implements ControlFunction<SwerveSample> {
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

    public AutoController() {
        rController.enableContinuousInput(-Math.PI, Math.PI);
        xController.close();
        yController.close();
        rController.close();
    }

    @Override
    public ChassisSpeeds apply(Pose2d pose, SwerveSample referenceState) {
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

        return out;
    }
}
