import static java.time.Duration.ofMillis;
import static java.time.Duration.ofMinutes;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeout;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.util.Random;
import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import com.igknighters.constants.FieldConstants;
import com.igknighters.util.AllianceFlip;

public class AllianceFlipTests {
    Random rnd = new Random();

    @Test
    @DisplayName("Test Flip Translation2d")
    public void testFlipTranslationd() {
        Translation2d randomTranslation = new Translation2d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0),
                rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0));

        assertEquals(
                new Translation2d(FieldConstants.FIELD_LENGTH - randomTranslation.getX(), randomTranslation.getY()),
                AllianceFlip.flipTranslation(randomTranslation));
    }

    @Test
    @DisplayName("Test Flip Translation3d")
    public void testFlipTranslation2d() {
        Translation3d randomTranslation = new Translation3d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0), rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0), rnd.nextDouble(6.0));

        assertEquals(
            new Translation3d(FieldConstants.FIELD_LENGTH - randomTranslation.getX(), randomTranslation.getY(), randomTranslation.getZ()),
            AllianceFlip.flipTranslation(randomTranslation)
        );
    }

    @Test
    @DisplayName("Test Flip Rotation2d")
    public void testFlipRotation2d() {
        Rotation2d randomRotation = Rotation2d.fromRadians(rnd.nextDouble((2.0 * Math.PI) + 1.0));

        assertEquals(
            Rotation2d.fromRadians(randomRotation.getRadians() + Math.PI),
            AllianceFlip.flipRotation(randomRotation)
        );
    }

    @Test
    @DisplayName("Test Flip Rotation3d")
    public void testFlipRotation3d() {
        Rotation3d randomRotation = new Rotation3d(rnd.nextDouble((2.0 * Math.PI) + 1.0), rnd.nextDouble((2.0 * Math.PI) + 1.0), rnd.nextDouble((2.0 * Math.PI) + 1.0));
        Rotation3d expectedRotation = new Rotation3d(randomRotation.getX(), randomRotation.getY(), randomRotation.getZ() + Math.PI);
        Rotation3d outputRotation = AllianceFlip.flipRotation(randomRotation);

        assertEquals(expectedRotation.getX(), outputRotation.getX());
        assertEquals(expectedRotation.getY(), outputRotation.getY());
        assertEquals(expectedRotation.getZ(), outputRotation.getZ());
    }

    @Test
    @DisplayName("Test Flip Pose2d")
    public void testFlipPose2d() {
        Pose2d randomPose = new Pose2d(
                new Translation2d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0),
                        rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0)),
                Rotation2d.fromDegrees(rnd.nextDouble(361)));

        assertEquals(
                new Pose2d(
                        new Translation2d(FieldConstants.FIELD_LENGTH - randomPose.getX(), randomPose.getY()),
                        Rotation2d.fromDegrees(randomPose.getRotation().getDegrees() + 180)),
                AllianceFlip.flipPose(randomPose));
    }

    @Test
    @DisplayName("Test Flip Pose3d")
    public void testFlipPose3d() {
        Pose3d randomPose = new Pose3d(
                new Translation3d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0),
                        rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0), 6.0),
                new Rotation3d(rnd.nextDouble((2.0 * Math.PI) + 1.0), rnd.nextDouble((2.0 * Math.PI) + 1.0),
                        rnd.nextDouble((2.0 * Math.PI) + 1.0)));
        Pose3d expectedPose = new Pose3d(
                new Translation3d(FieldConstants.FIELD_LENGTH - randomPose.getX(), randomPose.getY(),
                        randomPose.getZ()),
                new Rotation3d(randomPose.getRotation().getX(), randomPose.getRotation().getY(),
                        randomPose.getRotation().getZ() + Math.PI));
        Pose3d outputPose = AllianceFlip.flipPose(randomPose);

        assertEquals(expectedPose.getTranslation(), outputPose.getTranslation());
        assertEquals(expectedPose.getRotation().getX(), outputPose.getRotation().getX());
        assertEquals(expectedPose.getRotation().getY(), outputPose.getRotation().getY());
        assertEquals(expectedPose.getRotation().getZ(), outputPose.getRotation().getZ());
    }

    @Test
    @DisplayName("Test Flip Transform2d")
    public void testFlipTransform2d() {
        Transform2d randomTransform = new Transform2d(
                new Translation2d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0),
                        rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0)),
                Rotation2d.fromDegrees(rnd.nextDouble(361)));

        assertEquals(
                new Transform2d(
                        new Translation2d(FieldConstants.FIELD_LENGTH - randomTransform.getX(), randomTransform.getY()),
                        Rotation2d.fromDegrees(randomTransform.getRotation().getDegrees() + 180)),
                AllianceFlip.flipTransform(randomTransform));
    }

    @Test
    @DisplayName("Test Flip Transform3d")
    public void testFlipTransform3d() {
        Transform3d randomTransform = new Transform3d(
                new Translation3d(rnd.nextDouble(FieldConstants.FIELD_LENGTH + 1.0),
                        rnd.nextDouble(FieldConstants.FIELD_WIDTH + 1.0), 6.0),
                new Rotation3d(rnd.nextDouble((2.0 * Math.PI) + 1.0), rnd.nextDouble((2.0 * Math.PI) + 1.0),
                        rnd.nextDouble((2.0 * Math.PI) + 1.0)));
        Transform3d expectedPose = new Transform3d(
                new Translation3d(FieldConstants.FIELD_LENGTH - randomTransform.getX(), randomTransform.getY(),
                        randomTransform.getZ()),
                new Rotation3d(randomTransform.getRotation().getX(), randomTransform.getRotation().getY(),
                        randomTransform.getRotation().getZ() + Math.PI));
        Transform3d outputPose = AllianceFlip.flipTransform(randomTransform);

        assertEquals(expectedPose.getTranslation(), outputPose.getTranslation());
        assertEquals(expectedPose.getRotation().getX(), outputPose.getRotation().getX());
        assertEquals(expectedPose.getRotation().getY(), outputPose.getRotation().getY());
        assertEquals(expectedPose.getRotation().getZ(), outputPose.getRotation().getZ());
    }
}