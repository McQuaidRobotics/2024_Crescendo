package frc.robot.subsystems.super_structure;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.super_structure.States.SuperStructurePosition;
import frc.robot.util.ShuffleboardApi.ShuffleTab;

public class Visualizer {

        private final Mechanism2d mechanism;
        private final MechanismRoot2d rootCurrent, rootSetpoint;
        private final MechanismLigament2d elevatorCurrent, wristLowerCurrent, wristUpperCurrent;
        private final MechanismLigament2d elevatorSetpoint, wristLowerSetpoint, wristUpperSetpoint;

        private final static Double ELEVATOR_RANGE = Constants.kSuperStructure.Specs.ELEVATOR_MAX_METERS
                        - Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS;

        public Visualizer() {
                mechanism = new Mechanism2d(2.0, 2.0);

                // rootCurrent = mechanism.getRoot(
                // "Pivot Current",
                // Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
                // Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getZ());
                rootCurrent = mechanism.getRoot(
                                "Pivot Current",
                                0.0,
                                0.0);
                elevatorCurrent = rootCurrent.append(new MechanismLigament2d(
                                "Elevator Current",
                                Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS,
                                Constants.kSuperStructure.Specs.PIVOT_MIN_ANGLE));
                wristLowerCurrent = elevatorCurrent.append(new MechanismLigament2d(
                                "Wrist Lower Current",
                                0.32,
                                -15));
                wristUpperCurrent = elevatorCurrent.append(new MechanismLigament2d(
                                "Wrist Upper Current",
                                0.43,
                                20));

                // rootSetpoint = mechanism.getRoot(
                // "Pivot Setpoint",
                // Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getY(),
                // Constants.kSuperStructure.Specs.PIVOT_OFFSET_METERS.getZ());
                rootSetpoint = mechanism.getRoot(
                                "Pivot Setpoint",
                                0.0,
                                0.0);
                elevatorSetpoint = rootSetpoint.append(new MechanismLigament2d(
                                "Elevator Setpoint",
                                Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS,
                                Constants.kSuperStructure.Specs.PIVOT_MIN_ANGLE));
                wristLowerSetpoint = elevatorSetpoint.append(new MechanismLigament2d(
                                "Wrist Lower Setpoint",
                                0.32,
                                -15));
                wristUpperSetpoint = elevatorSetpoint.append(new MechanismLigament2d(
                                "Wrist Upper Setpoint",
                                0.43,
                                20));

                elevatorSetpoint.setColor(new Color8Bit(170, 180, 180));
                wristLowerSetpoint.setColor(new Color8Bit(170, 180, 180));
                wristUpperSetpoint.setColor(new Color8Bit(170, 180, 180));
                elevatorSetpoint.setLineWeight(3.0);
                wristLowerSetpoint.setLineWeight(3.0);
                wristUpperSetpoint.setLineWeight(3.0);
        }

        public void setShuffleboardTab(ShuffleTab tab) {
                tab.addSendable("Superstructure Visualizer", mechanism);
        }

        public void updateCurrent(SuperStructurePosition currentForm) {
                elevatorCurrent.setAngle(currentForm.pivotDegrees);
                elevatorCurrent.setLength(currentForm.elevatorMeters);

                // lerp the elevator color based on % of range
                // 0% = green, 100% = red
                Double percent = (currentForm.elevatorMeters - Constants.kSuperStructure.Specs.ELEVATOR_MIN_METERS)
                                / ELEVATOR_RANGE;
                int red = (int) (percent * 255);
                int green = (int) ((1 - percent) * 255);
                elevatorCurrent.setColor(new Color8Bit(red, green, 0));

                wristLowerCurrent.setAngle(currentForm.wristDegrees - 15);
                wristUpperCurrent.setAngle(currentForm.wristDegrees + 20);

                var intakeVolts = currentForm.intakeVoltage;
                Color8Bit intakeColor;
                if (intakeVolts > 0.1) {
                        intakeColor = new Color8Bit(0, 255, 100);
                } else if (intakeVolts < -0.1) {
                        intakeColor = new Color8Bit(255, 0, 100);
                } else {
                        intakeColor = new Color8Bit(127, 127, 100);
                }

                wristLowerCurrent.setColor(intakeColor);
                wristUpperCurrent.setColor(intakeColor);
        }

        public void updateSetpoint(SuperStructurePosition pose) {
                elevatorSetpoint.setAngle(pose.pivotDegrees);
                elevatorSetpoint.setLength(pose.elevatorMeters);
                wristLowerSetpoint.setAngle(pose.wristDegrees - 15);
                wristUpperSetpoint.setAngle(pose.wristDegrees + 20);
        }
}
