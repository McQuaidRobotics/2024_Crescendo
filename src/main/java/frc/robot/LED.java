package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LED {

    private static LED instance;
    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }
        return instance;
    }

    public enum LedPatterns {
        DISABLED(0),
        TELEOP(1),
        AUTO(2),
        TEST(3),
        CONE(4),
        CUBE(5),
        _30S_LEFT(6),
        PLACING(7),
        BOOTING(31);

        private final int value;

        public int getValue() {
            return value;
        }

        private LedPatterns(int value) {
            this.value = value;
        }
    }

    private final DigitalOutput bitOne = new DigitalOutput(10+10);
    private final DigitalOutput bitTwo = new DigitalOutput(8+10);
    private final DigitalOutput bitThree = new DigitalOutput(7+10);
    private final DigitalOutput bitFour = new DigitalOutput(9+10);

    private Timer timer;
    private Double duration;
    private LedPatterns pattern;
    private LedPatterns lastPattern;

    /** Creates a new LED. */
    public LED() {
        this.timer = new Timer();
        this.pattern = LedPatterns.DISABLED;
        this.lastPattern = LedPatterns.DISABLED;
        this.duration = 0.0;
        sendPattern(LedPatterns.DISABLED);

        new Trigger(DriverStation::isFMSAttached)
            .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
            .onTrue(new InstantCommand(() -> this.setLed(LedPatterns._30S_LEFT, 2)));
    }

    private void sendPattern(LedPatterns pattern) {
        int num = pattern.getValue();
        bitOne.set((num & 0b0001) == 0b0001);
        bitTwo.set((num & 0b0010) == 0b0010);
        bitThree.set((num & 0b0100) == 0b0100);
        bitFour.set((num & 0b1000) == 0b1000);
    }

    public void setLed(LedPatterns pattern, double seconds) {
        this.timer.restart();
        this.pattern = pattern;
        this.duration = seconds;
    }

    public void setLed(LedPatterns pattern) {
        setLed(pattern, Double.POSITIVE_INFINITY);
    }

    public void setDefault() {
        this.duration = 0.0;
        if (DriverStation.isDisabled()) {
            this.pattern = LedPatterns.DISABLED;
        } else if (DriverStation.isAutonomous()) {
            this.pattern = LedPatterns.AUTO;
        } else if (GamepieceMode.getDesiredPiece() == GamepieceMode.CONE) {
            this.pattern = LedPatterns.CONE;
        } else if (GamepieceMode.getDesiredPiece() == GamepieceMode.CUBE) {
            this.pattern = LedPatterns.CUBE;
        }
    }

    public void run() {
        if (timer.hasElapsed(duration)) {
            setDefault();
        }
        if (pattern != lastPattern || duration == 0) {
            sendPattern(pattern);
            lastPattern = pattern;
        }
    }
}
