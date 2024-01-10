package com.igknighters.subsystems.umbrella;

import com.igknighters.subsystems.umbrella.intake.*;
import com.igknighters.subsystems.umbrella.shooter.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Umbrella extends SubsystemBase {

    private final Intake intake;
    private final Shooter shooter;

    public Umbrella() {
        if (RobotBase.isSimulation()) {
            intake = new IntakeSim();
            shooter = new ShooterSim();
        } else {
            intake = new IntakeReal();
            shooter = new ShooterReal();
        }
    }

    @Override
    public void periodic() {
        intake.periodic();
        shooter.periodic();
    }
}
