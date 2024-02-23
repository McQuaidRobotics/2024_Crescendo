package com.igknighters.commands.stem;

import java.util.function.DoubleSupplier;

import com.igknighters.GlobalState;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kUmbrella;
import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;
import com.igknighters.subsystems.stem.StemSolvers;
import com.igknighters.util.AllianceFlip;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
     * at a target
     */
    private static class AimAtCommand extends Command {
        private Stem stem;
        private double pivotRads;
        private double stemLength;

        private AimAtCommand(Stem stem, double pivotRads, double telescopeMeters) {
            addRequirements(stem);
            this.stem = stem;
            this.pivotRads = pivotRads;
            this.stemLength = telescopeMeters;
        }

        @Override
        public void execute() {
            boolean blueAlliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue);
            Translation2d speaker = FieldConstants.SPEAKER.toTranslation2d();
            Translation2d targetTranslation = blueAlliance ? speaker : AllianceFlip.flipTranslation(speaker);

            ChassisSpeeds currentChassisSpeed = GlobalState.getFieldRelativeVelocity();

            Pose2d currentPose = GlobalState.getLocalizedPose();

            double distance = currentPose.getTranslation().getDistance(targetTranslation);

            Translation2d adjustedTarget = new Translation2d(
                    targetTranslation.getX() - (currentChassisSpeed.vxMetersPerSecond * (distance / kUmbrella.NOTE_VELO)),
                    targetTranslation.getY() - (currentChassisSpeed.vyMetersPerSecond * (distance / kUmbrella.NOTE_VELO)));

            double wristRads = StemSolvers.linearSolveWristTheta(
                    stemLength,
                    pivotRads,
                    currentPose.getTranslation().getDistance(adjustedTarget),
                    FieldConstants.SPEAKER.getZ());

            stem.setStemPosition(StemPosition.fromRadians(
                    pivotRads,
                    wristRads + pivotRads,
                    stemLength));
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
        return stem.run(() -> stem.setStemPosition(pose))
                .withName("Hold Stem(" + pose + ")");
    }

    /**
     * Will continously move the stem to target a point in space until the command
     * is canceled or overidden with a new command.
     * 
     * @param stem   The stem subsystem
     * @param target The point in space to aim at
     * @return A command to be scheduled
     */
    public static Command aimAt(Stem stem, double pivotRads, double telescopeMeters) {
        return new AimAtCommand(stem, pivotRads, telescopeMeters)
                .withName("Aim At(Speaker)");
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
