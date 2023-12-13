package frc.robot.commands.swerve;

import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConsts;
import frc.robot.commands.Helpers;

/** An example command that uses an example subsystem. */
public class TeleopSwerve2 extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Swerve swerve;
    private final DoubleSupplier translationXSup;
    private final DoubleSupplier translationYSup;
    private final DoubleSupplier rotationXSup;
    private final DoubleSupplier rotationYSup;

    public TeleopSwerve2(
            Swerve swerve,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotationX,
            DoubleSupplier rotationY) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationXSup = Helpers.deadbandSupplier(translationX, ControllerConsts.LEFT_DEADBAND);
        this.translationYSup = Helpers.deadbandSupplier(translationY, ControllerConsts.LEFT_DEADBAND);
        this.rotationXSup = Helpers.deadbandSupplier(rotationX, ControllerConsts.RIGHT_DEADBAND);
        this.rotationYSup = Helpers.deadbandSupplier(rotationY, ControllerConsts.RIGHT_DEADBAND);
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        translationXSup.getAsDouble(), translationYSup.getAsDouble()),
                new Translation2d(
                        rotationXSup.getAsDouble(), rotationYSup.getAsDouble()),
                true);
    }
}
