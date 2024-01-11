package com.igknighters.subsystems.swerve;

import com.igknighters.constants.ConstValues.kSwerve;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveVisualizer {

    private static class ModuleVisualizer {
        private static final double MAX_LENGTH = 5.0;

        Mechanism2d mechanism;
        MechanismLigament2d moduleLig;

        public ModuleVisualizer(int modNum) {
            mechanism = new Mechanism2d(MAX_LENGTH*2.2, MAX_LENGTH*2.2);

            moduleLig = new MechanismLigament2d(
                "Module " + modNum + " Visualizer",
                5.0,
                0.0,
                2.0,
                new Color8Bit(0, 0, 255)
            );

            mechanism
                .getRoot("azmith", MAX_LENGTH*1.1, MAX_LENGTH*1.1)
                .append(moduleLig);

        }

        public void update(SwerveModuleState state) {
            moduleLig.setAngle(state.angle);

            var length = (state.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY) * MAX_LENGTH;
            moduleLig.setLength(length);
        }
    }

    private final Swerve swerve;
    private final SwerveModule[] modules;
    private final ModuleVisualizer[] moduleVisual;
    private final Field2d field = new Field2d();

    public SwerveVisualizer(Swerve swerve, SwerveModule... modules) {
        this.swerve = swerve;
        this.modules = modules;

        moduleVisual = new ModuleVisualizer[this.modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleVisual[i] = new ModuleVisualizer(modules[i].getModuleNumber());
        }
    }

    public void update() {
        for (int i = 0; i < modules.length; i++) {
            moduleVisual[i].update(
                modules[i].getCurrentState()
            );
        }
        var pose = swerve.getPose();
        updateField(pose);
    }

    private void updateField(Pose2d roboPose) {
        field.setRobotPose(roboPose);

        var trans = roboPose.getTranslation();

        field.getObject("SwerveModules").setPoses(List.of(
            
        ));
    }
}
