import static java.time.Duration.ofMillis;
import static java.time.Duration.ofMinutes;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTimeout;
import static org.junit.jupiter.api.Assertions.assertTimeoutPreemptively;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.CountDownLatch;

import example.domain.Person;
import example.util.Calculator;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;

import com.igknighters.util.AllianceFlip;

public class AllianceFlipTests {

    @Test
    public void testFlipPose2d() {
        assertAll("testFlipPose2d",
            () -> assertEquals(new Pose2d(), AllianceFlip(new Pose2d()))
        );
    }
}