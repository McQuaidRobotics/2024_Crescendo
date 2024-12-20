package sham;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Torque;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Arrays;

import org.dyn4j.geometry.Vector2;
import sham.ShamArena.ShamEnvTiming;
import sham.configs.ShamSwerveConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.GeometryConvertor;
import sham.utils.mathutils.MeasureMath;
import sham.utils.mathutils.MassMath.PhysicsMass;
import sham.utils.mathutils.MeasureMath.XY;

public class ShamSwerve extends ShamDriveTrain {
    protected final ShamEnvTiming timing;
    protected final ShamSwerveModule[] moduleSimulations;
    protected final ShamGyro gyroSimulation;
    private final ShamSwerveConfig config;
    private final ShamRobot<ShamSwerve> robot;
    private final PhysicsMass chassisMass;
    private final SwerveDriveKinematics kinematics;

    /**
     * Creates a Swerve Drive Simulation.
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper dimensions, module
     * simulations, module translations, gyro simulation, and initial pose on the field.
     *
     * @param config a {@link ShamSwerveConfig} instance containing the configurations of * this drivetrain
     */
    ShamSwerve(ShamRobot<ShamSwerve> robot, ShamSwerveConfig config) {
        super(robot.logger.getSubLogger("Swerve"), config);
        this.robot = robot;
        this.timing = robot.timing();
        this.config = config;
        this.moduleSimulations = new ShamSwerveModule[config.moduleTranslations.length];
        final Force gravityForceOnEachModule = Newtons.of(config.robotMassKg * 9.8).div(moduleSimulations.length);
        for (int i = 0; i < moduleSimulations.length; i++) {
            moduleSimulations[i] = new ShamSwerveModule(robot, config, logger, i, gravityForceOnEachModule,
                    ShamMotorController.none(), ShamMotorController.none());
        }
        this.gyroSimulation = new ShamGyro(timing, config.gyroConfig);

        this.chassisMass = new PhysicsMass(Kilograms.of(config.robotMassKg), KilogramSquareMeters.of(config.robotMoI));
        this.kinematics = new SwerveDriveKinematics(config.moduleTranslations);

        RuntimeLog.debug("created swerve drive simulation");
    }

    @Override
    protected void simTick() {
        simulateModules();
        gyroSimulation.updateSimulationSubTick(RadiansPerSecond.of(chassis.getAngularVelocity()));
        super.simTick();
    }

    private void simulateModules() {
        final Rotation2d chassisRotation = getChassisWorldPose().getRotation();
        final ChassisSpeeds chassisSpeeds = this.getChassisWorldSpeeds();

        // apply propulsion forces to chassis
        for (final ShamSwerveModule module : moduleSimulations) {
            final Vector2 moduleWorldPosition = chassis
                    .getWorldPoint(GeometryConvertor.toDyn4jVector2(
                        module.translation().rotateBy(chassisRotation))
                    );
            chassis.applyForce(module.force(chassisRotation), moduleWorldPosition);
        }

        // sum up all the acceleration due to friction from each module
        LinearAcceleration xFrictionAccel = MetersPerSecondPerSecond.zero();
        LinearAcceleration yFrictionAccel = MetersPerSecondPerSecond.zero();
        AngularAcceleration angularFrictionAccel = RadiansPerSecondPerSecond.zero();
        for (int i = 0; i < moduleSimulations.length; i++) {
            final XY<Force> frictionForce = moduleSimulations[i].friction(
                chassisSpeeds,
                chassisRotation
            );
            final var pack = chassisMass.accelerationsDueToForce(
                frictionForce, XY.of(moduleSimulations[i].translation())
            );
            xFrictionAccel = xFrictionAccel.plus(pack.getFirst().x());
            yFrictionAccel = yFrictionAccel.plus(pack.getFirst().y());
            angularFrictionAccel = angularFrictionAccel.plus(pack.getSecond());

            logger.log("Friction/module" + i + "/frictionForce", frictionForce, XY.struct);
            logger.log("Friction/module" + i + "/xyFrictionAccel", pack.getFirst(), XY.struct);
            logger.log("Friction/module" + i + "/angularFrictionAccel", pack.getSecond());
        }

        // clamp the friction acceleration to prevent the robot from accelerating in the opposite direction
        final ChassisSpeeds wheelSpeeds = kinematics.toChassisSpeeds(
            Arrays.stream(moduleSimulations).map(ShamSwerveModule::state).toArray(SwerveModuleState[]::new)
        );
        final ChassisSpeeds unwantedSpeeds = wheelSpeeds.minus(chassisSpeeds);

        logger.log("Friction/wheelSpeeds", wheelSpeeds, ChassisSpeeds.struct);
        logger.log("Friction/unwantedSpeeds", unwantedSpeeds, ChassisSpeeds.struct);

        final LinearAcceleration xAccelNeededToStop = MeasureMath.negate(
            MetersPerSecond.of(unwantedSpeeds.vxMetersPerSecond))
            .div(timing.dt()
        );
        final LinearAcceleration yAccelNeededToStop = MeasureMath.negate(
            MetersPerSecond.of(unwantedSpeeds.vyMetersPerSecond))
            .div(timing.dt()
        );
        final AngularAcceleration angularAccelNeededToStop = MeasureMath.negate(
            RadiansPerSecond.of(unwantedSpeeds.omegaRadiansPerSecond))
            .div(timing.dt()
        );
        xFrictionAccel = MeasureMath.clamp(xFrictionAccel, xAccelNeededToStop);
        yFrictionAccel = MeasureMath.clamp(yFrictionAccel, yAccelNeededToStop);
        // angularFrictionAccel = MeasureMath.clamp(angularFrictionAccel, angularAccelNeededToStop);

        logger.log("Friction/xAccelNeededToStop", xAccelNeededToStop);
        logger.log("Friction/yAccelNeededToStop", yAccelNeededToStop);
        logger.log("Friction/angularAccelNeededToStop", angularAccelNeededToStop);
        logger.log("Friction/xFrictionAccel", xFrictionAccel);
        logger.log("Friction/yFrictionAccel", yFrictionAccel);
        logger.log("Friction/angularFrictionAccel", angularFrictionAccel);

        // convert the friction acceleration to forces and torques and apply them to the chassis
        final Force xFrictionForce = chassisMass.forceDueToAcceleration(xFrictionAccel);
        final Force yFrictionForce = chassisMass.forceDueToAcceleration(yFrictionAccel);
        final Torque angularFrictionTorque = chassisMass.torqueDueToAcceleration(angularFrictionAccel);
        chassis.applyForce(new Vector2(xFrictionForce.in(Newtons), yFrictionForce.in(Newtons)));
        chassis.applyTorque(angularFrictionTorque.in(NewtonMeters));
    }

    public ShamSwerveModule[] getModules() {
        return moduleSimulations;
    }

    public ShamGyro getGyro() {
        return this.gyroSimulation;
    }

    public ShamSwerve withSetModuleControllers(int moduleId, ShamMotorController driveController,
            ShamMotorController steerController) {
        moduleSimulations[moduleId].teardown();
        moduleSimulations[moduleId] = new ShamSwerveModule(
            robot,
            config,
            logger,
            moduleId,
            Newtons.of(config.robotMassKg * 9.8).div(moduleSimulations.length),
            driveController,
            steerController
        );
        return this;
    }

    public ShamEnvTiming timing() {
        return timing;
    }
}
