package igknighters.commands;

import igknighters.commands.stem.StemCommands;
import igknighters.commands.umbrella.UmbrellaCommands;
import igknighters.constants.ConstValues.kControls;
import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.stem.StemPosition;
import igknighters.subsystems.umbrella.Umbrella;
import igknighters.util.plumbing.TunableValues;
import igknighters.util.plumbing.TunableValues.TunableDouble;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShotMetricTesting {
    private static final TunableDouble shooterSpeed = TunableValues.getDouble("TestShooterSpeed", kControls.SHOOTER_RPM);
    private static final TunableDouble wristAngle = TunableValues.getDouble("TestWristAngle", Units.radiansToDegrees(StemPosition.STOW.wristRads));
    private static final TunableDouble pivotAngle = TunableValues.getDouble("TestPivotAngle", Units.radiansToDegrees(StemPosition.STOW.pivotRads));


    public static Command intakeStraightUp(Stem stem,  Umbrella umbrella) {
        return Commands.race(
            StemCommands.holdAt(stem, StemPosition.SHOOTER_TEST),
            UmbrellaCommands.intakeWhileIdleShooter(umbrella, shooterSpeed::value)
        ).andThen(
            StemCommands.holdAt(stem, StemPosition.SHOOTER_TEST)
                .alongWith(UmbrellaCommands.idleShooter(umbrella, shooterSpeed::value))
        );
    }

    public static Command shootStraightUp(Stem stem, Umbrella umbrella) {
        return Commands.race(
            StemCommands.holdAt(stem, StemPosition.SHOOTER_TEST),
            UmbrellaCommands.shoot(umbrella, shooterSpeed::value)
        ).andThen(
            StemCommands.holdAt(stem, StemPosition.SHOOTER_TEST)
                .alongWith(UmbrellaCommands.idleShooter(umbrella, shooterSpeed::value))
        );
    }

    public static StemPosition testStemPose() {
        return StemPosition.fromDegrees(
            pivotAngle.value(),
            wristAngle.value(),
            StemPosition.STOW.telescopeMeters
        );
    }

    public static Command intakeTest(Stem stem,  Umbrella umbrella) {
        return Commands.race(
            StemCommands.holdAt(stem, testStemPose()),
            UmbrellaCommands.intakeWhileIdleShooter(umbrella, shooterSpeed::value)
        ).andThen(
            StemCommands.holdAt(stem, testStemPose())
                .alongWith(UmbrellaCommands.idleShooter(umbrella, shooterSpeed::value))
        );
    }

    public static Command shootTest(Stem stem, Umbrella umbrella) {
        return Commands.race(
            StemCommands.holdAt(stem, testStemPose()),
            UmbrellaCommands.shoot(umbrella, shooterSpeed::value)
        ).andThen(
            StemCommands.holdAt(stem, testStemPose())
                .alongWith(UmbrellaCommands.idleShooter(umbrella, shooterSpeed::value))
        );
    }
}