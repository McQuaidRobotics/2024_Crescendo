package sham;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.MomentOfInertia;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import sham.ShamArena.ShamEnvTiming;
import sham.ShamMechanism.MechanismDynamics;
import sham.ShamMechanism.MechanismInputs;
import sham.ShamMechanism.MechanismOutputs;
import sham.configs.ShamSwerveConfig;
import sham.configs.ShamSwerveModuleConfig;
import sham.utils.RuntimeLog;
import sham.utils.geometry.Velocity2d;
import sham.utils.mathutils.MeasureMath;
import sham.utils.mathutils.MeasureMath.XY;

public class ShamSwerveModule {
    private final ShamRobot<ShamSwerve> robot;
    private final ShamMechanism driveMech;
    private final ShamMechanism steerMech;
    private final double wheelsCoefficientOfFriction;
    private final Force gravityForce;
    private final Distance wheelRadius;
    private final Translation2d translation;
    private final int moduleId;
    protected final DataLogger logger;

    private final ShamEnvTiming timing;

    ShamSwerveModule(
            ShamRobot<ShamSwerve> robot,
            ShamSwerveConfig config,
            DataLogger logger,
            int moduleId,
            Force gravityForce,
            Supplier<MomentOfInertia> rotorInertia,
            ShamMotorController driveController,
            ShamMotorController steerController) {
        this.logger = logger.getSubLogger("SwerveModule" + moduleId);
        this.robot = robot;
        final ShamSwerveModuleConfig moduleConfig = config.swerveModuleConfig;
        timing = robot.timing();
        this.gravityForce = gravityForce;
        translation = config.moduleTranslations[moduleId];
        driveMech = new ShamMechanism(
                "SwerveModuleDriveMotor" + moduleId,
                moduleConfig.driveConfig.motor,
                driveController,
                moduleConfig.driveConfig.rotorInertia,
                moduleConfig.driveConfig.gearRatio,
                moduleConfig.driveConfig.friction,
                new MechanismDynamics() {
                    @Override
                    public MomentOfInertia extraInertia() {
                        return rotorInertia.get();
                    }
                },
                moduleConfig.driveConfig.limits,
                moduleConfig.driveConfig.noise,
                timing);
        steerMech = new ShamMechanism(
                "SwerveModuleSteerMotor" + moduleId,
                moduleConfig.steerConfig.motor,
                steerController,
                moduleConfig.steerConfig.rotorInertia,
                moduleConfig.steerConfig.gearRatio,
                moduleConfig.steerConfig.friction,
                MechanismDynamics.zero(),
                moduleConfig.steerConfig.limits,
                moduleConfig.steerConfig.noise,
                timing);
        robot.addMechanism(driveMech);
        robot.addMechanism(steerMech);
        wheelsCoefficientOfFriction = config.swerveModuleConfig.tireCoefficientOfFriction;
        wheelRadius = Meters.of(config.swerveModuleConfig.wheelsRadiusMeters);
        this.moduleId = moduleId;

        RuntimeLog.debug("Created a swerve module simulation");
    }

    public record ModuleMotorPair<T>(T drive, T steer) {
    }

    public ModuleMotorPair<MechanismInputs> inputs() {
        return new ModuleMotorPair<>(driveMech.inputs(), steerMech.inputs());
    }

    public ModuleMotorPair<MechanismOutputs> outputs() {
        return new ModuleMotorPair<>(driveMech.outputs(), steerMech.outputs());
    }

    public SwerveModuleState state() {
        return new SwerveModuleState(
                driveMech.outputs().velocity().in(RadiansPerSecond) * wheelRadius.in(Meters),
                new Rotation2d(steerMech.outputs().position()));
    }

    protected void teardown() {
        robot.removeMechanism(driveMech);
        robot.removeMechanism(steerMech);
    }

    protected Translation2d translation() {
        return translation;
    }

    protected int id() {
        return moduleId;
    }

    protected XY<Force> force(final Rotation2d robotHeading) {
        final Rotation2d steerMechAngle = new Rotation2d(steerMech.outputs().position())
                .plus(robotHeading);
        logger.log("worldAngle", steerMechAngle, Rotation2d.struct);
        final Force gripForce = gravityForce.times(wheelsCoefficientOfFriction);
        final Force driveMechAppliedForce = driveMech.inputs().torque().div(wheelRadius);

        final boolean isSkidding = MeasureMath.abs(driveMechAppliedForce).gt(gripForce);
        final Force propellingForce;
        if (isSkidding) {
            propellingForce = gripForce.times(MeasureMath.signum(driveMechAppliedForce));
        } else {
            propellingForce = driveMechAppliedForce;
        }

        logger.log("isSkidding", isSkidding);
        logger.log("propellingForce", propellingForce);

        return new XY<Force>(
                propellingForce.times(steerMechAngle.getCos()),
                propellingForce.times(steerMechAngle.getSin())
        );
    }

    protected XY<Force> friction(ChassisSpeeds chassisSpeeds, Rotation2d robotHeading) {
        final Force gripForce = gravityForce.times(wheelsCoefficientOfFriction);

        final double tangentialVelocity = chassisSpeeds.omegaRadiansPerSecond * translation().getNorm();
        final Velocity2d tangentialVelocityVector = new Velocity2d(
                robotHeading.getCos() * tangentialVelocity,
                robotHeading.getSin() * tangentialVelocity);
        final Velocity2d moduleWorldVelocity = new Velocity2d(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond);
        final Velocity2d moduleDriveVelocity = new Velocity2d(
                driveMech.outputs().velocity().in(RadiansPerSecond) * wheelRadius.in(Meters),
                new Rotation2d(steerMech.outputs().position()).plus(robotHeading));
        final Velocity2d unwantedVelocity = moduleWorldVelocity
                .plus(tangentialVelocityVector)
                .minus(moduleDriveVelocity);

        logger.log("tangentialVelocity", tangentialVelocity);
        logger.log("tangentialVelocityVector", tangentialVelocityVector, Velocity2d.struct);
        logger.log("moduleWorldVelocity", moduleWorldVelocity, Velocity2d.struct);
        logger.log("moduleDriveVelocity", moduleDriveVelocity, Velocity2d.struct);
        logger.log("unwantedVelocity", unwantedVelocity, Velocity2d.struct);

        return new XY<Force>(
                gripForce.times(-Math.signum(unwantedVelocity.getVX())),
                gripForce.times(-Math.signum(unwantedVelocity.getVY()))
        );
    }
}
