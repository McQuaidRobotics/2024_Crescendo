package com.igknighters.commands.stem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kControls;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.constants.ConstValues.kStem.kTelescope;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.subsystems.stem.StemSolvers.AimSolveStrategy;
import com.igknighters.subsystems.stem.StemSolvers.StemSolveInput;
import com.igknighters.util.geom.AllianceFlip;
import com.igknighters.util.plumbing.TunableValues;
import com.igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import monologue.Monologue;
import edu.wpi.first.wpilibj2.command.Command;

public class StemCommands {
    /**
     * A command class that continually calculates the wrist radians needed to aim
     * at the speaker
     */
    private static class AimAtSpeakerCommand extends Command {
        private final Stem stem;
        private final AimSolveStrategy aimStrategy;
        private final Supplier<Pose2d> poseSupplier;
        private final Supplier<ChassisSpeeds> velocitySupplier;
        private final boolean canFinish;

        private Translation2d targetTranslation;
        private boolean hasFinished = false;

        private AimAtSpeakerCommand(
            Stem stem,
            AimSolveStrategy aimStrategy,
            boolean canFinish,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> velocitySupplier
        ) {
            addRequirements(stem);
            this.stem = stem;
            this.aimStrategy = aimStrategy;
            this.canFinish = canFinish;
            this.poseSupplier = poseSupplier;
            this.velocitySupplier = velocitySupplier;
        }

        @Override
        public void initialize() {
            Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
            targetTranslation = AllianceFlip.isBlue() ? speaker : AllianceFlip.flipTranslation(speaker);
        }

        @Override
        public void execute() {
            ChassisSpeeds currentChassisSpeed = velocitySupplier.get();
            Pose2d currentPose = poseSupplier.get();

            double targetDistance = currentPose.getTranslation().getDistance(targetTranslation);

            Translation2d adjustedTarget = new Translation2d(
                    targetTranslation.getX()
                            - (currentChassisSpeed.vxMetersPerSecond * (targetDistance / kUmbrella.NOTE_VELO)),
                    targetTranslation.getY()
                            - (currentChassisSpeed.vyMetersPerSecond * (targetDistance / kUmbrella.NOTE_VELO)));

            Translation2d lookaheadTranslation = currentPose.getTranslation().plus(new Translation2d(
                currentChassisSpeed.vxMetersPerSecond * kControls.SOTM_LOOKAHEAD_TIME,
                currentChassisSpeed.vyMetersPerSecond * kControls.SOTM_LOOKAHEAD_TIME
            ));

            double distance = lookaheadTranslation.getDistance(adjustedTarget);
            stem.log("aim/distance", distance);

            StemSolveInput solveInput = new StemSolveInput(
                stem.getStemPosition(),
                StemPosition.fromRadians(
                    kControls.STATIONARY_AIM_AT_PIVOT_RADIANS,
                    kControls.STATIONARY_WRIST_ANGLE,
                    kTelescope.MIN_METERS
                ),
                distance,
                FieldConstants.SPEAKER.getZ(),
                kUmbrella.NOTE_VELO
            );

            hasFinished = stem.gotoStemPosition(
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
        private static final TunableDouble noteMps = TunableValues.getDouble("Aim/NoteMps", 11.0);

        private final Stem stem;
        private final Translation2d passPoint;
        private final Supplier<Pose2d> poseSupplier;
        private final boolean canFinish;

        private Translation2d targetTranslation;
        private boolean hasFinished = false;

        private AimAtPassPointCommand(
            Stem stem,
            Translation2d passPoint,
            boolean canFinish,
            Supplier<Pose2d> poseSupplier
        ) {
            addRequirements(stem);
            this.stem = stem;
            this.passPoint = passPoint;
            this.canFinish = canFinish;
            this.poseSupplier = poseSupplier;
        }

        @Override
        public void initialize() {
            targetTranslation = AllianceFlip.isBlue() ? passPoint : AllianceFlip.flipTranslation(passPoint);
        }

        @Override
        public void execute() {
            Pose2d currentPose = poseSupplier.get();
            double distance = currentPose.getTranslation().getDistance(targetTranslation);
            Monologue.log("Aim/PassDistance", distance);

            double pivotRads = kControls.STATIONARY_PASS_PIVOT_RADIANS;
            double telescopeMeters = kControls.STATIONARY_PASS_TELESCOPE_METERS;

            double wristRads = StemSolvers.gravitySolveWristTheta2(
                telescopeMeters,
                pivotRads,
                currentPose.getTranslation(),
                new Translation3d(
                    targetTranslation.getX(),
                    targetTranslation.getY(),
                    0.0
                ),
                noteMps.value(),
                false,
                true
            );

            stem.gotoStemPosition(
                StemPosition.fromRadians(
                    pivotRads,
                    wristRads,
                    telescopeMeters
                )
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
        return moveTo(stem, pose, 1.0);
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
        return stem.run(() -> stem.gotoStemPosition(pose))
            .until(() -> stem.isAt(pose, toleranceMult))
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
        return stem.run(() -> stem.gotoStemPosition(pose, 0.0))
                .withName("Hold Stem(" + pose + ")");
    }

    /**
     * Will command the stem to hold still at its current position.
     * 
     * @param stem The stem subsystem
     * @return A command to be scheduled
     */
    public static Command holdStill(Stem stem) {
        return stem.run(() -> stem.gotoStemPosition(stem.getStemPosition(), 0.0))
                .withName("Hold Stem Still");
    }

    /**
     * Aims the pivot or wrist or both depending on the aim strategy.
     * 
     * @param stem      The stem subsystem
     * @param canFinish Whether the command can finish
     * @param poseSupplier Localizer
     * @return A command to be scheduled
     */
    public static Command aimAtSpeaker(Stem stem, boolean canFinish, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> veloSupplier) {
        return aimAtSpeaker(stem, kControls.DEFAULT_AIM_STRATEGY, canFinish, poseSupplier, veloSupplier);
    }

    /**
     * Aims the pivot or wrist or both depending on the default aim
     * strategy in constants.
     * 
     * @param stem         The stem subsystem
     * @param aimStrategy  The aiming strategy to use when targeting the speaker
     * @param canFinish    Whether or not the command can finish
     * @param poseSupplier Localizer
     * @return A command to be scheduled
     */
    public static Command aimAtSpeaker(Stem stem, AimSolveStrategy aimStrategy, boolean canFinish, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> veloSupplier) {
        return new AimAtSpeakerCommand(stem, aimStrategy, canFinish, poseSupplier, veloSupplier)
                .withName("AimAtSpeaker(" + aimStrategy.name() + ")");
    }

    /**
     * Aims the whole stem 
     * @param stem
     * @param passPoint
     * @param time
     * @param canFinish
     * @param poseSupplier
     * @return
     */
    public static Command aimAtPassPoint(Stem stem, Translation2d passPoint, boolean canFinish, Supplier<Pose2d> poseSupplier) {
        return new AimAtPassPointCommand(stem, passPoint, canFinish, poseSupplier)
            .withName(String.format("AimAtPassPoint(x: %.2f, y: %.2f)", passPoint.getX(), passPoint.getY()));
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
            Stem stem,
            DoubleSupplier pivotPercentOut,
            DoubleSupplier telescopePercentOut,
            DoubleSupplier wristPercentOut
    ) {
        return stem.run(() -> {
            stem.setStemVolts(
                    pivotPercentOut.getAsDouble() * 12.0,
                    wristPercentOut.getAsDouble() * 12.0,
                    telescopePercentOut.getAsDouble() * 12.0);
        }).withName("Test Stem");
    }
}
