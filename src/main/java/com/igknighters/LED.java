package com.igknighters;

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
        DISABLED,
        TELEOP,
        AUTO,
        TEST,
        _20S_LEFT,
        PLACING,
        BOOTING;

        private LedPatterns() {
        }
    }

    private Timer timer;
    private double duration;
    private LedPatterns pattern;
    private LedPatterns lastPattern;

    public LED() {
        this.timer = new Timer();
        this.pattern = LedPatterns.DISABLED;
        this.lastPattern = LedPatterns.DISABLED;
        this.duration = 0.0;
        sendPattern(LedPatterns.DISABLED);

        new Trigger(DriverStation::isFMSAttached)
                .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
                .onTrue(new InstantCommand(() -> this.setLed(LedPatterns._20S_LEFT, 2)));
    }

    private void sendPattern(LedPatterns pattern) {
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
        } else {
            this.pattern = LedPatterns.TELEOP;
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
