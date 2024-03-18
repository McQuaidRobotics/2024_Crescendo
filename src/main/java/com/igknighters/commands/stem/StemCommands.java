package com.igknighters.commands.stem;

import java.util.function.DoubleSupplier;

import com.igknighters.GlobalState;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.constants.ConstValues.kUmbrella.kShooter;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.subsystems.stem.StemSolvers.AimSolveStrategy;
import com.igknighters.subsystems.stem.StemSolvers.StemSolveInput;
import com.igknighters.util.geom.AllianceFlip;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import monologue.MonoDashboard;
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
     * A command class that continually calculates the wrist radians needed to aim
     * at the speaker
     */
    private static class AimAtSpeakerCommand extends Command {
        private Stem stem;
        private AimSolveStrategy aimStrategy;

        private boolean hasFinished = false, canFinish = false;

        private AimAtSpeakerCommand(Stem stem, AimSolveStrategy aimStrategy, boolean canFinish) {
            addRequirements(stem);
            this.stem = stem;
            this.aimStrategy = aimStrategy;
            this.canFinish = canFinish;
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
            MonoDashboard.put("Aim/distance", distance);

            StemSolveInput solveInput = new StemSolveInput(
                stem.getStemPosition(),
                StemPosition.fromRadians(
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    kControls.STATIONARY_WRIST_ANGLE,
                    kTelescope.MIN_METERS
                ),
                distance,
                FieldConstants.SPEAKER.getZ(),
                kShooter.RPM_TO_AVERAGE_NOTE_VELO_CURVE.lerp((kShooter.DISTANCE_TO_RPM_CURVE.lerp(distance)))
            );

            hasFinished = stem.setStemPosition(
                aimStrategy.solve(solveInput)
            );
        }

        @Override
        public boolean isFinished() {
            return hasFinished && canFinish;
        }
    }

    /**
     * A command class that continually calculates the wrist radians needed to pass a gamepiece to a position on then field
     */
    private static class AimAtPassPointCommand extends Command {
        private Stem stem;
        private double time;
        private Translation2d passPoint;
        private boolean hasFinished = false, canFinish = false;

        private AimAtPassPointCommand(Stem stem, Translation2d passPoint, double time, boolean canFinish) {
            addRequirements(stem);
            this.stem = stem;
            this.time = time;
            this.passPoint = passPoint;
            this.canFinish = canFinish;
        }

        @Override
        public void execute() {
            Translation2d passPoint = this.passPoint;
            Translation2d targetTranslation = AllianceFlip.isBlue() ? passPoint : AllianceFlip.flipTranslation(passPoint);

            Pose2d currentPose = GlobalState.getLocalizedPose();
            double distance = currentPose.getTranslation().getDistance(targetTranslation);

            double pivotRads = kControls.STATIONARY_AIM_AT_PIVOT_RADIANS;
            double telescopeMeters = kTelescope.MIN_METERS;
            double wristRads = StemSolvers.passWristSolveTheta(
                telescopeMeters,
                pivotRads,
                distance,
                kUmbrella.NOTE_VELO,
                time,
                true
            ); 

            hasFinished = stem.setStemPosition(
                StemPosition.fromRadians(pivotRads, wristRads, telescopeMeters)
            );
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
    public static Command aimAtSpeaker(Stem stem, AimSolveStrategy aimStrategy, boolean canFinish) {
        return new AimAtSpeakerCommand(stem, aimStrategy, canFinish)
                .withName("Aim At SPEAKER");
    }

    public static Command aimAtPassPoint(Stem stem, Translation2d passPoint, double time, boolean canFinish) {
        return new AimAtPassPointCommand(stem, passPoint, time, canFinish)
            .withName("Aim At Pass Point");
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
}
