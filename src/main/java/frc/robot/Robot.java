package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.Autos;
import frc.robot.util.ShuffleboardApi;

public class Robot extends TimedRobot {

    private Command autoCmd;
    private Autos.AutoRoutines autoRoutine;
    private Alliance alliance = Alliance.Invalid;
    private final SendableChooser<Autos.AutoRoutines> autoRoutineChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        RobotContainer.RobotContainerInit();

        Autos.AutoRoutines[] autoRoutines = Autos.AutoRoutines.values();
        for (Autos.AutoRoutines autoRoutine : autoRoutines) {
            if (autoRoutine == Autos.AutoRoutines.NOTHING) {
                autoRoutineChooser.setDefaultOption(autoRoutine.name(), autoRoutine);
                continue;
            }
            autoRoutineChooser.addOption(autoRoutine.name(), autoRoutine);
        }

        SmartDashboard.putString("AutoCommand", autoCmd == null ? "null" : autoCmd.getName());
        ShuffleboardApi.getTab("Autos")
            .addSendable("Autonomous Routine", autoRoutineChooser);

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.logNetworkTables(true);
    }

    @Override
    public void robotPeriodic() {
        ShuffleboardApi.run();
        CommandScheduler.getInstance().run();
        LED.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        var selectedRoutine = autoRoutineChooser.getSelected();
        var currAlliance = DriverStation.getAlliance();
        if (selectedRoutine != autoRoutine || alliance != currAlliance) {
            autoRoutine = selectedRoutine;
            alliance = currAlliance;
            autoCmd = autoRoutine.getCommand();
            SmartDashboard.putString("AutoCommand", autoCmd == null ? "null" : autoCmd.getName());
        }
    }

    @Override
    public void autonomousInit() {
        if (autoCmd != null) {
            autoCmd.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
