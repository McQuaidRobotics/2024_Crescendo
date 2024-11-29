package org.ironmaple;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.SimArena.SimulationTiming;
import org.ironmaple.configs.SwerveConfig;
import org.ironmaple.utils.RuntimeLog;
import org.ironmaple.utils.mathutils.GeometryConvertor;


public class SimDriveTrainSwerve extends SimDriveTrain {
    protected final SimulationTiming timing;
    protected final SimDriveTrainSwerveModule[] moduleSimulations;
    protected final SimGyro gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;
    private final double gravityForceOnEachModule;
    private final SwerveConfig config;
    private final SimRobot robot;

    /**
     *
     *
     * <h2>Creates a Swerve Drive Simulation.</h2>
     *
     * <p>This constructor initializes a swerve drive simulation with the given robot mass, bumper dimensions, module
     * simulations, module translations, gyro simulation, and initial pose on the field.
     *
     * @param config a {@link SwerveConfig} instance containing the configurations of * this drivetrain
     */
    public SimDriveTrainSwerve(SimRobot robot, SwerveConfig config) {
        super(config);
        this.robot = robot;
        this.timing = robot.timing();
        this.config = config;
        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = new SimDriveTrainSwerveModule[config.moduleTranslations.length];
        for (int i = 0; i < moduleSimulations.length; i++) {
            moduleSimulations[i] = new SimDriveTrainSwerveModule(robot, config, i, SimMotorController.none(), SimMotorController.none());
        }
        this.gyroSimulation = new SimGyro(timing, config.gyroConfig);

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

    public SimDriveTrainSwerveModule[] getModules() {
        return moduleSimulations;
    }

    public SimGyro getGyro() {
        return this.gyroSimulation;
    }

    public SimDriveTrainSwerve withSetModuleControllers(int moduleId, SimMotorController driveController, SimMotorController steerController) {
        moduleSimulations[moduleId] = new SimDriveTrainSwerveModule(robot, config, moduleId, driveController, steerController);
        return this;
    }
}
