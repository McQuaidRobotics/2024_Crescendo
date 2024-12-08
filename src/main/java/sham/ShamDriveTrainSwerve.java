package sham;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.dyn4j.geometry.Vector2;
import sham.ShamArena.ShamEnvTiming;
import sham.configs.ShamSwerveConfig;
import sham.utils.RuntimeLog;
import sham.utils.mathutils.GeometryConvertor;


public class ShamDriveTrainSwerve extends ShamDriveTrain {
    protected final ShamEnvTiming timing;
    protected final ShamDriveTrainSwerveModule[] moduleSimulations;
    protected final ShamGyro gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;
    private final double gravityForceOnEachModule;
    private final ShamSwerveConfig config;
    private final ShamRobot robot;

    /**
     *
     *
     * <h2>Creates a Swerve Drive Simulation.</h2>
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper dimensions, module
     * simulations, module translations, gyro simulation, and initial pose on the field.
     *
     * @param config a {@link ShamSwerveConfig} instance containing the configurations of * this drivetrain
     */
    public ShamDriveTrainSwerve(ShamRobot robot, ShamSwerveConfig config) {
        super(config);
        this.robot = robot;
        this.timing = robot.timing();
        this.config = config;
        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = new ShamDriveTrainSwerveModule[config.moduleTranslations.length];
        for (int i = 0; i < moduleSimulations.length; i++) {
            moduleSimulations[i] = new ShamDriveTrainSwerveModule(robot, config, i, ShamMotorController.none(), ShamMotorController.none());
        }
        this.gyroSimulation = new ShamGyro(timing, config.gyroConfig);

        this.kinematics = new SwerveDriveKinematics(moduleTranslations);

        this.gravityForceOnEachModule = (config.robotMassKg * 9.8) / moduleSimulations.length;

        chassis.setLinearDamping(1.5);
        chassis.setAngularDamping(1.5);

        RuntimeLog.debug("created swerve drive simulation");
    }

    @Override
    protected void simTick() {
        simulateModulePropellingForces();

        gyroSimulation.updateSimulationSubTick(RadiansPerSecond.of(chassis.getAngularVelocity()));
    }

    private void simulateModulePropellingForces() {
        for (int i = 0; i < moduleSimulations.length; i++) {
            final Vector2 moduleWorldPosition = chassis.getWorldPoint(GeometryConvertor.toDyn4jVector2(moduleTranslations[i]));
            final Vector2 moduleForce = moduleSimulations[i].moduleForce(
                gravityForceOnEachModule
            );
            chassis.applyForce(moduleForce, moduleWorldPosition);
        }
    }

    public ShamDriveTrainSwerveModule[] getModules() {
        return moduleSimulations;
    }

    public ShamGyro getGyro() {
        return this.gyroSimulation;
    }

    public ShamDriveTrainSwerve withSetModuleControllers(int moduleId, ShamMotorController driveController, ShamMotorController steerController) {
        moduleSimulations[moduleId] = new ShamDriveTrainSwerveModule(robot, config, moduleId, driveController, steerController);
        return this;
    }
}
