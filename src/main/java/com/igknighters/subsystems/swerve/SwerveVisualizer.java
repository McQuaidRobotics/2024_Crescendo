package com.igknighters.subsystems.swerve;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.subsystems.swerve.module.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveVisualizer {

    private static class ModuleVisualizer {
        private static final double MAX_LENGTH = 5.0;

        private static final Color8Bit MIN_COLOR = new Color8Bit(0, 255, 0);
        private static final Color8Bit MAX_COLOR = new Color8Bit(255, 0, 0);

        Mechanism2d mechanism;
        MechanismLigament2d moduleLig;

        public ModuleVisualizer(int modNum) {
            mechanism = new Mechanism2d(MAX_LENGTH * 2.2, MAX_LENGTH * 2.2);

            moduleLig = new MechanismLigament2d(
                    "Module " + modNum + " Visualizer",
                    5.0,
                    0.0,
                    2.0,
                    new Color8Bit(0, 0, 255));

            mechanism
                    .getRoot("azmith", MAX_LENGTH * 1.1, MAX_LENGTH * 1.1)
                    .append(moduleLig);

        }

        public Mechanism2d getMechanism() {
            return mechanism;
        }

        public void update(SwerveModuleState state) {
            moduleLig.setAngle(state.angle);

            var percent = (state.speedMetersPerSecond / kSwerve.MAX_DRIVE_VELOCITY);

            var length = percent * (MAX_LENGTH * 0.9) + (MAX_LENGTH * 0.1);
            moduleLig.setLength(length);

            var color = new Color8Bit(
                    (int) (Math.abs(percent) * (MAX_COLOR.red - MIN_COLOR.red) + MIN_COLOR.red),
                    (int) (Math.abs(percent) * (MAX_COLOR.green - MIN_COLOR.green) + MIN_COLOR.green),
                    (int) (Math.abs(percent) * (MAX_COLOR.blue - MIN_COLOR.blue) + MIN_COLOR.blue));

            moduleLig.setColor(color);
        }
    }

    private final SwerveModule[] modules;
    private final ModuleVisualizer[] moduleVisual;
    private final NetworkTable table;

    public SwerveVisualizer(SwerveModule... modules) {
        this.modules = modules;

        table = NetworkTableInstance.getDefault().getTable("Visualizers");

        moduleVisual = new ModuleVisualizer[this.modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleVisual[i] = new ModuleVisualizer(modules[i].getModuleNumber());
            moduleVisual[i]
                    .getMechanism()
                    .initSendable(
                            getBuilder(
                                    "SwerveModules/Module[" + modules[i].getModuleNumber() + "]"));
        }

    }

    private SendableBuilderImpl getBuilder(String subtable) {
        var builder = new SendableBuilderImpl();
        builder.setTable(table.getSubTable(subtable));
        return builder;
    }

    // OBJ_COUNT: 8
    public void update() {
        for (int i = 0; i < modules.length; i++) {
            moduleVisual[i].update(
                    modules[i].getCurrentState());
        }
    }
}
