package com.igknighters.commands.swerve.teleop;

import com.igknighters.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.controllers.ControllerParent;

public class TeleopSwerveBase extends Command {
    /**
     * Orient the chassis speeds for the user,
     * If real robot this means up on translation stick moves away from driver.
     * If simulation and {@link kSwerve#ORIENT_TELEOP_FOR_SIM} is true,
     * it will make up on the translation stick move up on the field visualization.
     * If simulation and {@link kSwerve#ORIENT_TELEOP_FOR_SIM} is false it will replicate the real robot.
     * @param input The controller input
     * @return The adjusted controller input
     */
    @SuppressWarnings("unused")
    public static Translation2d orientForUser(Translation2d input) {
        if (RobotBase.isSimulation() && kSwerve.ORIENT_TELEOP_FOR_SIM) {
            return new Translation2d(
                    input.getX(),
                    -input.getY());
        } else {
            Translation2d chassisSpeedsAdj = input.rotateBy(Rotation2d.fromDegrees(-90));
            return new Translation2d(-chassisSpeedsAdj.getX(), chassisSpeedsAdj.getY());
        }
    }

    protected final Swerve swerve;

    private final DoubleSupplier rawTranslationXSup;
    private final DoubleSupplier rawTranslationYSup;
    private final DoubleSupplier rawRotationXSup;
    private final DoubleSupplier rawRotationYSup;

    public TeleopSwerveBase(Swerve swerve, ControllerParent controller) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.rawTranslationXSup = controller.leftStickX();
        this.rawTranslationYSup = controller.leftStickY();
        this.rawRotationXSup = controller.rightStickX();
        this.rawRotationYSup = controller.rightStickY();
    }

    protected double getTranslationX() {
        // inverted because left is positive for field due to Y being increased but left
        // is negative for controller
        var v = -kSwerve.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawTranslationXSup.getAsDouble());
        SmartDashboard.putNumber("getTranslationX", v);
        return v;
    }

    protected double getTranslationY() {
        var v = kSwerve.TELEOP_TRANSLATION_AXIS_CURVE.lerpKeepSign(rawTranslationYSup.getAsDouble());
        SmartDashboard.putNumber("getTranslationY", v);
        return v;
    }

    protected double getRotationX() {
        // inverted because left is positive for field due to Y being increased but left
        // is negative for controller
        var v = -kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawRotationXSup.getAsDouble());
        SmartDashboard.putNumber("getRotationX", v);
        return v;
    }

    protected double getRotationY() {
        var v = kSwerve.TELEOP_ROTATION_AXIS_CURVE.lerpKeepSign(rawRotationYSup.getAsDouble());
        SmartDashboard.putNumber("getRotationY", v);
        return v;
    }

    public static class TeleopSwerveOmni extends Command {
        public static enum TeleopMode {
            TRADITIONAL,
            ABS_ROT,
            TARGET
        }

        private final SendableChooser<TeleopMode> chooser = new SendableChooser<>();

        private final Swerve swerve;
        private final ControllerParent controller;

        private TeleopMode mode = TeleopMode.TRADITIONAL;
        private TeleopSwerveBase currentCmd;

        public TeleopSwerveOmni(Swerve swerve, ControllerParent controller) {
            addRequirements(swerve);

            this.swerve = swerve;
            this.controller = controller;
            currentCmd = new TeleopSwerveTraditional(swerve, controller);

            chooser.setDefaultOption("Traditional", TeleopMode.TRADITIONAL);
            chooser.addOption("Absolute Rotation", TeleopMode.ABS_ROT);
            chooser.addOption("Target", TeleopMode.TARGET);

            SmartDashboard.putData("Teleop Mode", chooser);
        }

        @Override
        public void execute() {
            var newMode = chooser.getSelected();
            if (newMode != mode) {
                mode = newMode;
                switch (mode) {
                    case TRADITIONAL:
                        currentCmd = new TeleopSwerveTraditional(swerve, controller);
                        break;
                    case ABS_ROT:
                        currentCmd = new TeleopSwerveAbsRot(swerve, controller);
                        break;
                    case TARGET:
                        currentCmd = new TeleopSwerveTarget(swerve, controller);
                        break;
                }
            }
            currentCmd.execute();
        }
    }
}
