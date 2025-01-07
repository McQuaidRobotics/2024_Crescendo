package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.constants.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class GamepieceSimulator {
  private static final Translation2d[] AUTO_NOTES = {
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.175, 7.0),
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.175, 5.54),
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.175, 4.1),
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.825, 7.0),
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.825, 5.54),
    new Translation2d(FieldConstants.FIELD_LENGTH * 0.825, 4.1),
    new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, 7.44),
    new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, 5.77),
    new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, 4.11),
    new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, 2.42),
    new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, 0.76),
  };

  private static final Translation2d[] TELEOP_NOTES = {
    new Translation2d(16.0, 1.0), new Translation2d(15.2, 0.7)
  };

  private static final double NOTE_RADIUS = Units.inchesToMeters(7.0);

  /**
   * Exposes an easy api for tracking intaken notes in sim auto
   *
   * <p>This is a very memory inefficient way of solving this problem, this is ok because this util
   * should only be used in sim, if you want to use this on a roborio for whatever reason expect
   * high performance overhead
   *
   * @param poseSupplier A lambda that returns the current position of the robot (blue alliance
   *     origin)
   * @param isIntakingSupplier A lambda that returns if the robot is currently intaking
   * @param noteDisplayFunction A lambda that is given the current note positions on the field as
   *     translations
   * @param intakePointTransform A list of the robot relative transforms of the intake when the
   *     robot is intaking
   * @return A Trigger that will have a guranteed rising edge for every note the robot intakes
   */
  public static Trigger setupGamepieceAcquisitionSim(
      Supplier<Pose2d> poseSupplier,
      BooleanSupplier isIntakingSupplier,
      Consumer<List<Translation2d>> noteDisplayFunction,
      List<Transform2d> intakePointTransform) {

    AtomicBoolean didJustIntake = new AtomicBoolean(false);
    ArrayList<Translation2d> notes = new ArrayList<>();

    // auto sim notes
    Command autoCmd =
        new FunctionalCommand(
                () -> {
                  notes.clear();
                  notes.addAll(List.of(AUTO_NOTES));
                  noteDisplayFunction.accept(notes);
                },
                () -> {
                  // make sure every note intake causes an edge. If we are intaking multiple notes
                  // at once somehow
                  // only take 1 out of the array, use 1 cycle to register the rising edge then the
                  // next cycle to register the falling edge,
                  // then the next note can be intaken.
                  if (didJustIntake.get()) {
                    // registers the falling edge and prevents the array from being modified
                    didJustIntake.set(false);
                    return;
                  }
                  List<Translation2d> intakePoints =
                      intakePointTransform.stream()
                          .map(
                              transform ->
                                  poseSupplier.get().transformBy(transform).getTranslation())
                          .toList();
                  if (isIntakingSupplier.getAsBoolean()) {
                    List<Translation2d> newNotes =
                        notes.stream()
                            .filter(
                                note -> {
                                  if (didJustIntake.get()) {
                                    return true;
                                  }
                                  for (Translation2d intakePoint : intakePoints) {
                                    if (note.getDistance(intakePoint) < NOTE_RADIUS) {
                                      didJustIntake.set(true);
                                      return false;
                                    }
                                  }
                                  return true;
                                })
                            .toList();
                    notes.clear();
                    notes.addAll(newNotes);
                    noteDisplayFunction.accept(notes);
                  }
                },
                interrupted -> noteDisplayFunction.accept(List.of()),
                () -> false)
            .withName("GamepieceSimulatorAuto");
    RobotModeTriggers.autonomous().whileTrue(autoCmd);

    // teleop sim notes
    Command teleopCmd =
        new FunctionalCommand(
                () -> {
                  notes.clear();
                  notes.addAll(
                      List.of(TELEOP_NOTES).stream()
                          .map(t -> AllianceFlip.isBlue() ? t : AllianceFlip.flipTranslation(t))
                          .toList());
                  noteDisplayFunction.accept(notes);
                },
                () -> {
                  // make sure every note intake causes an edge. If we are intaking multiple notes
                  // at once somehow
                  // only take 1 out of the array, use 1 cycle to register the rising edge then the
                  // next cycle to register the falling edge,
                  // then the next note can be intaken.
                  if (didJustIntake.get()) {
                    // registers the falling edge and prevents the array from being modified
                    didJustIntake.set(false);
                    return;
                  }
                  List<Translation2d> intakePoints =
                      intakePointTransform.stream()
                          .map(
                              transform ->
                                  poseSupplier.get().transformBy(transform).getTranslation())
                          .toList();
                  if (isIntakingSupplier.getAsBoolean()) {
                    notes.stream()
                        .forEach(
                            note -> {
                              if (didJustIntake.get()) {
                                return;
                              }
                              for (Translation2d intakePoint : intakePoints) {
                                if (note.getDistance(intakePoint) < NOTE_RADIUS) {
                                  didJustIntake.set(true);
                                  return;
                                }
                              }
                            });
                  }
                },
                interrupted -> noteDisplayFunction.accept(List.of()),
                () -> false)
            .withName("GamepieceSimulatorTeleop");
    RobotModeTriggers.teleop().whileTrue(teleopCmd);

    return new Trigger(didJustIntake::get);
  }
}
