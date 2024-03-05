package com.igknighters.commands.stem;

import java.util.function.DoubleSupplier;

import com.igknighters.GlobalState;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kStem;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kStem.kWrist;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.util.geom.AllianceFlip;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class StemCommands {

    /**
     * A command class to assist in storing the done state of the stem movement
     * to be used in the finished check.
     */
    private static class MoveToCommand extends Command {
        private boolean isDone = false;
        private final StemPosition pose;
        private final Stem stem;
        private final double tolerance;

        private MoveToCommand(Stem stem, StemPosition pose, double tolerance) {
            addRequirements(stem);
            this.stem = stem;
            this.pose = pose;
            this.tolerance = tolerance;
        }

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            isDone = stem.setStemPosition(pose, tolerance);
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    /**
     * Allows for limited manual control of the pivot and telescope.
     */
    private static class LimitedManualControlCommand extends Command {
        private final double minPivotRads = Math.toRadians(10.0);
        private final double maxPivotRads = Math.toRadians(94.0);
        private final double minTelescopeMeters = kTelescope.MIN_METERS;
        private final double maxTelescopeMeters = kTelescope.MAX_METERS;
        
        private Stem stem;
        private DoubleSupplier leftStickYSup, rightStickYSup;
        private double deadband;

        private LimitedManualControlCommand(
            Stem stem, 
            DoubleSupplier leftStickYSup, 
            DoubleSupplier rightStickYSup,
            double deadband) {

            addRequirements(stem);
            this.stem = stem;
            this.leftStickYSup = leftStickYSup;
            this.rightStickYSup = rightStickYSup;
            this.deadband = deadband;
        }

        @Override
        public void initialize() {
            GlobalState.setClimbing(true);
        }

        @Override
        public void execute() {
            double leftY = MathUtil.applyDeadband(leftStickYSup.getAsDouble(), deadband);
            double rightY = -1.0 * MathUtil.applyDeadband(rightStickYSup.getAsDouble(), deadband);

            if (stem.getStemPosition().getPivotRads() <= minPivotRads) leftY = MathUtil.clamp(leftY, 0.0, 1.0);
            else if (stem.getStemPosition().getPivotRads() >= maxPivotRads) leftY = MathUtil.clamp(leftY, -1.0, 0.0);

            if (stem.getStemPosition().getTelescopeMeters() <= minTelescopeMeters) rightY = MathUtil.clamp(rightY, 0.0, 1.0);
            else if (stem.getStemPosition().getTelescopeMeters() >= maxTelescopeMeters) rightY = MathUtil.clamp(rightY, -1.0, 0.0);

            stem.setStemVolts(
                leftY * RobotController.getBatteryVoltage(), 
                0.0, 
                rightY * RobotController.getBatteryVoltage());
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interupted) {
            stem.stopMechanisms();
            GlobalState.setClimbing(false);
        }
    } 

    public enum AimStrategy {
        STATIONARY_WRIST,
        STATIONARY_PIVOT,
        MAX_HEIGHT,
        LOWEST_HEIGHT
    }

    /**
     * A command class that continually calculates the wrist radians needed to aim at the speaker
     */
    private static class AimAtSpeakerCommand extends Command {
        private Stem stem;
        private AimStrategy aimStrategy;

        private boolean hasFinished = false, canFinish = false;

        private AimAtSpeakerCommand(Stem stem, AimStrategy aimStrategy, boolean canFinish) {
            addRequirements(stem);
            this.stem = stem;
            this.aimStrategy = aimStrategy;
            this.canFinish = canFinish;
        }

        private StemPosition stationaryWristSolve(double distance, double wristRads) {
            double pivotRads = StemSolvers.linearSolvePivotTheta(
                    kTelescope.MIN_METERS,
                    wristRads,
                    distance,
                    FieldConstants.SPEAKER.getZ());

            return StemPosition.fromRadians(
                    pivotRads,
                    kControls.STATIONARY_WRIST_ANGLE,
                    kTelescope.MIN_METERS);
        }

        private StemPosition stationaryPivotSolveGravity(double distance) {
            double wristRads = StemSolvers.gravitySolveWristTheta(
                    kTelescope.MIN_METERS,
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    distance,
                    FieldConstants.SPEAKER.getZ(),
                    kUmbrella.NOTE_VELO);

            return StemPosition.fromRadians(
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    MathUtil.clamp(wristRads + kControls.STATIONARY_AIM_AT_PIVOT_RADIANS, kWrist.MIN_ANGLE,
                            kWrist.MAX_ANGLE),
                    kTelescope.MIN_METERS);
        }

        private StemPosition stationaryPivotSolve(double distance) {
            double wristRads = StemSolvers.linearSolveWristTheta(
                    kTelescope.MIN_METERS,
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    distance,
                    FieldConstants.SPEAKER.getZ());

            return StemPosition.fromRadians(
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    MathUtil.clamp(wristRads + kControls.STATIONARY_AIM_AT_PIVOT_RADIANS, kWrist.MIN_ANGLE,
                            kWrist.MAX_ANGLE),
                    kTelescope.MIN_METERS);
        }

        private StemPosition maxHeightSolve(double distance) {
            double wristRads = StemSolvers.linearSolveWristTheta(
                    kControls.MAX_HEIGHT_AIM_AT_TELESCOPE_METERS,
                    kControls.MAX_HEIGHT_AIM_AT_PIVOT_RADIANS,
                    distance,
                    FieldConstants.SPEAKER.getZ());

            return StemPosition.fromRadians(
                    kControls.MAX_HEIGHT_AIM_AT_PIVOT_RADIANS,
                    wristRads,
                    kControls.MAX_HEIGHT_AIM_AT_TELESCOPE_METERS);
        }

        private StemPosition movingBothSolve(double distance) {
            final double max = 45.0;
            final double min = 20.0;

            double t = MathUtil.clamp((distance - 2.5), 0.0, 6.0) / 6.0;
            double pivotRads = ((max - min) * t) + min;

            double wristRads = StemSolvers.linearSolveWristTheta(
                    kTelescope.MIN_METERS,
                    pivotRads,
                    distance,
                    FieldConstants.SPEAKER.getZ());

            return StemPosition.fromRadians(
                    pivotRads,
                    MathUtil.clamp(wristRads + pivotRads, kWrist.MIN_ANGLE,
                            kWrist.MAX_ANGLE),
                    kTelescope.MIN_METERS);
        }

        @Override
        public void execute() {
            Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
            Translation2d targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);

            ChassisSpeeds currentChassisSpeed = GlobalState.getFieldRelativeVelocity();

            Pose2d currentPose = GlobalState.getLocalizedPose();

            double targetDistance = currentPose.getTranslation().getDistance(targetTranslation);

            Translation2d adjustedTarget = new Translation2d(
                    targetTranslation.getX()
                            - (currentChassisSpeed.vxMetersPerSecond * (targetDistance / kUmbrella.NOTE_VELO)),
                    targetTranslation.getY()
                            - (currentChassisSpeed.vyMetersPerSecond * (targetDistance / kUmbrella.NOTE_VELO)));

            double distance = currentPose.getTranslation().getDistance(adjustedTarget);

            if (aimStrategy.equals(AimStrategy.STATIONARY_WRIST)) {
                hasFinished = stem.setStemPosition(stationaryWristSolve(distance, stem.getStemPosition().wristRads));
            } else if (aimStrategy.equals(AimStrategy.STATIONARY_PIVOT)) {
                var pose = stationaryPivotSolve(distance);
                SmartDashboard.putNumber("Aim/Linear", Units.radiansToDegrees(pose.getWristRads()));
                SmartDashboard.putNumber("Aim/Gravity", Units.radiansToDegrees(stationaryPivotSolveGravity(distance).getWristRads()));
                hasFinished = stem.setStemPosition(pose);
            } else if (aimStrategy.equals(AimStrategy.MAX_HEIGHT)) {
                hasFinished = stem.setStemPosition(maxHeightSolve(distance));
            } else if (aimStrategy.equals(AimStrategy.LOWEST_HEIGHT)) {
                hasFinished = stem.setStemPosition(movingBothSolve(distance));
            }
        }

        @Override
        public boolean isFinished() {
            return hasFinished && canFinish;
        }
    }

    /**
     * Will move the stem to a position and finish when it has reached
     * the desired position.
     * 
     * @param stem The stem subsystem
     * @param pose The desired pose
     * @return A command to be scheduled
     */
    public static Command moveTo(Stem stem, StemPosition pose) {
        return new MoveToCommand(stem, pose, 1.0)
                .withName("Move Stem(" + pose + ")");
    }

    /**
     * Will move the stem to a position and finish when it has reached
     * the desired position.
     * 
     * @param stem          The stem subsystem
     * @param pose          The desired pose
     * @param toleranceMult A value to multiply the accepted positional tolerance by
     * @return A command to be scheduled
     */
    public static Command moveTo(Stem stem, StemPosition pose, double toleranceMult) {
        return new MoveToCommand(stem, pose, toleranceMult)
                .withName("Move Stem(" + pose + ")");
    }

    /**
     * Will move the stem to a position but never finishes and has to be interupted.
     * 
     * @param stem The stem subsystem
     * @param pose The desired pose
     * @return A command to be scheduled
     */
    public static Command holdAt(Stem stem, StemPosition pose) {
        return stem.run(() -> stem.setStemPosition(pose, 0.0))
                .withName("Hold Stem(" + pose + ")");
    }

    /**
     * Aims the pivot or wrist or both depending on the aim strategy.
     * 
     * @param stem      The stem subsystem
     * @param canFinish Whether the command can finish
     * @return A command to be scheduled
     */
    public static Command aimAtSpeaker(Stem stem, boolean canFinish) {
        return new AimAtSpeakerCommand(stem, kControls.DEFAULT_AIM_STRATEGY, canFinish)
                .withName("Aim At SPEAKER");
    }

    /**
     * Aims the pivot or wrist or both depending on the default aim
     * strategy in constants.
     * 
     * @param stem        The stem subsystem
     * @param aimStrategy The aiming strategy to use when targeting the speaker
     * @return A command to be scheduled
     */
    public static Command aimAtSpeaker(Stem stem, AimStrategy aimStrategy, boolean canFinish) {
        return new AimAtSpeakerCommand(stem, aimStrategy, canFinish)
                .withName("Aim At SPEAKER");
    }

    /**
     * Allows manual control of the output of each individual component of the stem.
     * 
     * @param stem                The stem subsystem
     * @param pivotPercentOut     A supplier for the pivot motor output
     * @param telescopePercentOut A supplier for the telescope motor output
     * @param wristPercentOut     A supplier for the wrist motor output
     * @return A command to be scheduled
     */
    public static Command testStem(
            Stem stem, DoubleSupplier pivotPercentOut,
            DoubleSupplier telescopePercentOut, DoubleSupplier wristPercentOut) {
        return stem.run(() -> {
            stem.setStemVolts(
                    pivotPercentOut.getAsDouble() * 12.0,
                    wristPercentOut.getAsDouble() * 12.0,
                    telescopePercentOut.getAsDouble() * 12.0);
        }).withName("Test Stem");
    }

    /**
     * 
     * Allows for limited manual control of the pivot and telescope on the stem.
     * 
     * @param stem              The stem subsystem.
     * @param leftStickYSup     A supplier for the pivot motor output
     * @param rightStickYSup    A supplier for the telescope motor output
     * @param deadband          A deadband to apply to the motor outputs
     * @return                  A command to be scheduled
     */
    public static Command LimitedManualControl(Stem stem, DoubleSupplier leftStickYSup, DoubleSupplier rightStickYSup, double deadband) {
        return new LimitedManualControlCommand(
            stem, 
            leftStickYSup, 
            rightStickYSup, 
            deadband).withName("LimitedManualControl");
    }
}
