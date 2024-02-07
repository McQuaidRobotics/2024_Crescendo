package com.igknighters.commands.autos;

import com.igknighters.constants.ConstValues;
import com.igknighters.constants.FieldConstants;
import com.igknighters.constants.ConstValues.kAuto;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.Swerve;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Base pathfinding command */
public class SimplePathfindingCommand extends Command {
    private static int instances = 0;

    private final Timer timer = new Timer();
    private final Pose2d targetPose;
    private final GoalEndState goalEndState;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Consumer<ChassisSpeeds> output;
    private final PathFollowingController controller;
    private final double rotationDelayDistance;
    private final ReplanningConfig replanningConfig;
    private final Optional<Supplier<Rotation2d>> rotationOverrideSupplier = Optional.empty();
    private final Swerve swerve;

    private Optional<PathPlannerPath> currentPathOpt;
    private Optional<PathPlannerTrajectory> currentTrajectoryOpt;
    private Pose2d startingPose;

    private double timeOffset = 0;

    private boolean invalid = false;

    private SimplePathfindingCommand(
            Pose2d targetPose,
            double goalEndVel,
            double rotationDelayDistance,
            PathConstraints constraints,
            Optional<Supplier<Rotation2d>> rotationOverrideSupplier,
            Swerve swerve) {
        addRequirements(swerve);
        this.swerve = swerve;

        Pathfinding.ensureInitialized();

        validateTargetPose(targetPose);

        this.targetPose = targetPose;
        this.goalEndState = new GoalEndState(goalEndVel, targetPose.getRotation(), true);
        this.constraints = constraints;
        this.controller = new PPHolonomicDriveController(
                kAuto.AUTO_TRANSLATION_PID,
                kAuto.AUTO_ANGULAR_PID,
                ConstValues.PERIODIC_TIME,
                kSwerve.MAX_DRIVE_VELOCITY,
                kSwerve.DRIVEBASE_RADIUS);
        this.poseSupplier = swerve::getPose;
        this.speedsSupplier = swerve::getChassisSpeed;
        this.output = chassisSpeeds -> swerve.drive(
                chassisSpeeds, false);
        this.rotationDelayDistance = rotationDelayDistance;
        this.replanningConfig = kAuto.DYNAMIC_REPLANNING_CONFIG;

        instances++;
        HAL.report(tResourceType.kResourceType_PathFindingCommand, instances);
    }

    /**
     * Constructs a new base pathfinding command that will generate a path towards
     * the given pose.
     *
     * @param targetPose the pose to pathfind to
     * @param swerve     The swerve subsystem to use for pathfinding
     */
    public SimplePathfindingCommand(Pose2d targetPose, Swerve swerve) {
        this(targetPose, 0.0, 0.05, kAuto.DYNAMIC_PATH_CONSTRAINTS, Optional.empty(), swerve);
    }

    public SimplePathfindingCommand withEndVelo(double velo) {
        return new SimplePathfindingCommand(
                targetPose,
                velo,
                rotationDelayDistance,
                constraints,
                rotationOverrideSupplier,
                swerve);
    }

    public SimplePathfindingCommand withRotDelayDist(double rotationDelayDistance) {
        return new SimplePathfindingCommand(
                targetPose,
                goalEndState.getVelocity(),
                rotationDelayDistance,
                constraints,
                rotationOverrideSupplier,
                swerve);
    }

    public SimplePathfindingCommand withConstraints(PathConstraints constraints) {
        return new SimplePathfindingCommand(
                targetPose,
                goalEndState.getVelocity(),
                rotationDelayDistance,
                constraints,
                rotationOverrideSupplier,
                swerve);
    }

    public SimplePathfindingCommand withRotationOverride(Supplier<Rotation2d> rotationOverrideSupplier) {
        return new SimplePathfindingCommand(
                targetPose,
                goalEndState.getVelocity(),
                rotationDelayDistance,
                constraints,
                Optional.of(rotationOverrideSupplier),
                swerve);
    }

    public SimplePathfindingCommand withReplanningConfig(ReplanningConfig replanningConfig) {
        return new SimplePathfindingCommand(
                targetPose,
                goalEndState.getVelocity(),
                rotationDelayDistance,
                constraints,
                rotationOverrideSupplier,
                swerve);
    }

    @Override
    public void initialize() {
        if (invalid) {
            return;
        }

        if (rotationOverrideSupplier.isPresent()) {
            PPHolonomicDriveController
                    .setRotationTargetOverride(() -> Optional.of(rotationOverrideSupplier.get().get()));
        }

        currentTrajectoryOpt = Optional.empty();
        timeOffset = 0;

        Pose2d currentPose = poseSupplier.get();

        controller.reset(currentPose, speedsSupplier.get());

        Pathfinding.setStartPosition(currentPose.getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());

        startingPose = currentPose;
    }

    @Override
    public void execute() {
        if (invalid) {
            return;
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        PathPlannerLogging.logCurrentPose(currentPose);
        PPLibTelemetry.setCurrentPose(currentPose);

        // Skip new paths if we are close to the end
        boolean skipUpdates = currentTrajectoryOpt.isPresent()
                && currentPose
                        .getTranslation()
                        .getDistance(currentTrajectoryOpt.get().getEndState().positionMeters) < 2.0;

        if (!skipUpdates && Pathfinding.isNewPathAvailable()) {
            currentPathOpt = Optional.ofNullable(Pathfinding.getCurrentPath(constraints, goalEndState));

            if (currentPathOpt.isPresent()) {
                var currentPath = currentPathOpt.get();

                currentTrajectoryOpt = Optional
                        .ofNullable(new PathPlannerTrajectory(currentPath, currentSpeeds, currentPose.getRotation()));

                var currentTrajectory = currentTrajectoryOpt.get();

                // Find the two closest states in front of and behind robot
                int farStateIdx = 1;
                List<State> states = currentTrajectory.getStates();
                double farDistance = 0.0;
                double nextDistance = 0.0;
                while (nextDistance < farDistance) {
                    farDistance = states.get(Math.min(farStateIdx, states.size() - 1)).positionMeters
                            .getDistance(currentPose.getTranslation());

                    nextDistance = states.get(Math.min(farStateIdx + 1, states.size() - 1)).positionMeters
                            .getDistance(currentPose.getTranslation());

                    if (nextDistance < farDistance) {
                        farStateIdx++;
                    }
                }

                // Use the closest 2 states to interpolate what the time offset should be
                // This will account for the delay in pathfinding
                State closeState = states.get(Math.min(farStateIdx - 1, states.size() - 1));
                State farState = states.get(Math.min(farStateIdx, states.size() - 1));

                ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds,
                        currentPose.getRotation());
                Rotation2d currentHeading = new Rotation2d(
                        fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
                Rotation2d headingError = currentHeading.minus(closeState.heading);
                boolean onHeading = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 1.0
                        || Math.abs(headingError.getDegrees()) < 45;

                // Replan the path if our heading is off
                if (onHeading || !replanningConfig.enableInitialReplanning) {
                    double d = closeState.positionMeters.getDistance(farState.positionMeters);
                    double t = (currentPose.getTranslation().getDistance(closeState.positionMeters)) / d;
                    t = MathUtil.clamp(t, 0.0, 1.0);

                    timeOffset = GeometryUtil.doubleLerp(closeState.timeSeconds, farState.timeSeconds, t);

                    // If the robot is stationary and at the start of the path, set the time offset
                    // to the
                    // next loop
                    // This can prevent an issue where the robot will remain stationary if new paths
                    // come in
                    // every loop
                    if (timeOffset <= 0.02
                            && Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.1) {
                        timeOffset = 0.02;
                    }
                } else {
                    currentPathOpt = Optional.ofNullable(currentPath.replan(currentPose, currentSpeeds));
                    currentTrajectoryOpt = Optional.ofNullable(
                            new PathPlannerTrajectory(
                                    currentPathOpt.get(),
                                    currentSpeeds,
                                    currentPose.getRotation()));

                    timeOffset = 0;
                }

                if (currentPathOpt.isPresent()) {
                    PathPlannerLogging.logActivePath(currentPathOpt.get());
                    PPLibTelemetry.setCurrentPath(currentPathOpt.get());
                } else {
                    PathPlannerLogging.logActivePath(null);
                    PPLibTelemetry.setCurrentPath(null);
                }
            }

            timer.reset();
            timer.start();
        }

        if (currentTrajectoryOpt.isPresent()) {
            var currentTrajectory = currentTrajectoryOpt.get();

            PathPlannerTrajectory.State targetState = currentTrajectory.sample(timer.get() + timeOffset);

            if (replanningConfig.enableDynamicReplanning) {
                double previousError = Math.abs(controller.getPositionalError());
                double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

                if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
                        || currentError - previousError >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
                    replanPath(currentPose, currentSpeeds);
                    timer.reset();
                    timeOffset = 0.0;
                    targetState = currentTrajectory.sample(0);
                }
            }

            // Set the target rotation to the starting rotation if we have not yet traveled
            // the rotation
            // delay distance
            if (currentPose.getTranslation().getDistance(startingPose.getTranslation()) < rotationDelayDistance) {
                targetState.targetHolonomicRotation = startingPose.getRotation();
            }

            ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, targetState);

            double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

            PPLibTelemetry.setCurrentPose(currentPose);
            PathPlannerLogging.logCurrentPose(currentPose);

            if (controller.isHolonomic()) {
                PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
                PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());
            } else {
                PPLibTelemetry.setTargetPose(targetState.getDifferentialPose());
                PathPlannerLogging.logTargetPose(targetState.getDifferentialPose());
            }

            PPLibTelemetry.setVelocities(
                    currentVel,
                    targetState.velocityMps,
                    currentSpeeds.omegaRadiansPerSecond,
                    targetSpeeds.omegaRadiansPerSecond);
            PPLibTelemetry.setPathInaccuracy(controller.getPositionalError());

            output.accept(targetSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        if (invalid) {
            return true;
        }

        if (currentTrajectoryOpt.isPresent()) {
            return timer.hasElapsed(currentTrajectoryOpt.get().getTotalTimeSeconds() - timeOffset);
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (invalid) {
            return;
        }

        if (rotationOverrideSupplier.isPresent()) {
            PPHolonomicDriveController.setRotationTargetOverride(Optional::empty);
        }

        timer.stop();

        if (!interrupted && goalEndState.getVelocity() < 0.1) {
            output.accept(new ChassisSpeeds());
        }
        if (interrupted) {
            output.accept(new ChassisSpeeds());
        }

        PathPlannerLogging.logActivePath(null);
    }

    private void replanPath(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        PathPlannerPath replanned = currentPathOpt.get().replan(currentPose, currentSpeeds);
        currentTrajectoryOpt = Optional
                .ofNullable(new PathPlannerTrajectory(replanned, currentSpeeds, currentPose.getRotation()));
        PathPlannerLogging.logActivePath(replanned);
        PPLibTelemetry.setCurrentPath(replanned);
    }

    private void validateTargetPose(Pose2d targetPose) {
        var x = targetPose.getTranslation().getX();
        var y = targetPose.getTranslation().getY();
        boolean xTooLarge = x > FieldConstants.FIELD_LENGTH;
        boolean xTooSmall = x < 0;
        boolean yTooLarge = y > FieldConstants.FIELD_WIDTH;
        boolean yTooSmall = y < 0;
        if (xTooLarge || xTooSmall || yTooLarge || yTooSmall) {
            String message = "Target pose is out of bounds for  "
                    + this.getName()
                    + " Pathfinding Command. "
                    + "Because ";
            if (xTooLarge) {
                message += "x is too large, ";
            }
            if (xTooSmall) {
                message += "x is too small, ";
            }
            if (yTooLarge) {
                message += "y is too large, ";
            }
            if (yTooSmall) {
                message += "y is too small, ";
            }
            message = message.substring(0, message.length() - 2);
            DriverStation.reportError(message, false);
            invalid = true;
        }
    }
}
