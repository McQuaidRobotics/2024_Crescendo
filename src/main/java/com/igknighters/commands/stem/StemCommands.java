package com.igknighters.commands.stem;

import java.util.function.DoubleSupplier;

import com.igknighters.subsystems.stem.Stem;
import com.igknighters.subsystems.stem.StemPosition;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
     * @param stem The stem subsystem
     * @param pose The desired pose
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
     * Will continously move the stem to target a point in space.
     * @param stem The stem subsystem
     * @param target The point in space to aim at
     * @return A command to be scheduled
     */
    public static Command aimAt(Stem stem, Translation3d target, double rpm) {
        //TODO
        return Commands.none();
    }

    /**
     * Allows manual control of the output of each individual component of the stem.
     * 
     * @param stem The stem subsystem
     * @param pivotPercentOut A supplier for the pivot motor output
     * @param telescopePercentOut A supplier for the telescope motor output
     * @param wristPercentOut A supplier for the wrist motor output
     * @return A command to be scheduled
     */
    public static Command testStem(
        Stem stem, DoubleSupplier pivotPercentOut,
        DoubleSupplier telescopePercentOut, DoubleSupplier wristPercentOut
    ) {
        return stem.run(() -> {
            stem.setStemVolts(
                pivotPercentOut.getAsDouble() * 12.0,
                wristPercentOut.getAsDouble() * 12.0,
                telescopePercentOut.getAsDouble() * 12.0
            );
        }).withName("Test Stem");
    }
}
