package com.igknighters.commands.autos;

import java.util.Optional;

import com.igknighters.Robot;
import com.igknighters.subsystems.swerve.Swerve;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import monologue.MonoDashboard;

public class Autos {
    private static SendableChooser<Command> autoChooser;
    private static Optional<Command> autoCmdOverride = Optional.empty();

    /**
     * Creates the sendable chooser to be used for path planner autos
     * 
     * @param swerve The swerve subsystem to be used in dynamic autos
     */
    public static void createSendableChooser(Swerve swerve) {
        autoChooser = AutoBuilder.buildAutoChooser("No Auto");
        for (Command dynamicAutoCmd : DynamicRoutines.choosableDynamicRoutines(swerve)) {
            autoChooser.addOption("(Dynamic) " + dynamicAutoCmd.getName(), dynamicAutoCmd);
        }
        MonoDashboard.publishSendable("AutoChooser", autoChooser);
    }

    public static Command getAutonomousCommand() {
        if (autoCmdOverride.isPresent() && Robot.isUnitTest())
            return autoCmdOverride.get();
        if (autoChooser == null)
            return new InstantCommand().withName("Nothing");
        return autoChooser.getSelected();
    }

    public static String getSelectedAutoName() {
        return getAutonomousCommand().getName();
    }

    /**
     * Only to be used in unit tests, will force
     * {@link Autos#getAutonomousCommand()}
     * to return a specific command.
     * Supplying null as a command to this function will return
     * {@link Autos#getAutonomousCommand()} to default behavior;
     * 
     * @param cmd The command to force {@link Autos#getAutonomousCommand()} to
     *            return
     */
    public static void setAutoOverrideTest(Command cmd) {
        autoCmdOverride = Optional.ofNullable(cmd);
    }
}
