package com.igknighters;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.CAN;
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

    private static Animation animDisabled = new ColorFlowAnimation(255, 0, 0, 0, 0.2, 8, Direction.Forward);
    private static Animation animTeleOp = new FireAnimation(1.0, 0.2, 8, 0.3, 0.3);
    private static Animation animAuto = new RainbowAnimation(1.0, 0.2, 8);
    private static Animation animTest = new RgbFadeAnimation(1.0, 0.2, 8);
    private static Animation anim20sLeft = new SingleFadeAnimation(255, 255, 255);
    private static Animation animShooting = new TwinkleOffAnimation(255, 255, 0);
    private static Animation animBooting = new StrobeAnimation(255, 0, 0, 0, 0.2, 8);
    
    public enum LedAnimations {
        DISABLED(animDisabled),
        TELEOP(animTeleOp),
        AUTO(animAuto),
        TEST(animTest),
        _20S_LEFT(anim20sLeft),
        SHOOTING(animShooting),
        BOOTING(animBooting);

        private final Animation anim;

        public Animation getAnim(){
            return anim;
        }
        
        private LedAnimations(Animation anim) {
            this.anim=anim;
        }
    }
    
    


    private Timer timer;
    private Double duration;
    private LedAnimations animation;
    private LedAnimations lastPattern;
    private CANdle candle;
    CANdleConfiguration config;

    public LED() {
        this.timer = new Timer();
        this.animation = LedAnimations.DISABLED;
        this.lastPattern = LedAnimations.DISABLED;
        this.duration = 0.0;
        candle = new CANdle(0);
        config = new CANdleConfiguration();
        config.v5Enabled = true;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;
        candle.setLEDs(255, 0, 0);
        sendAnimation(LedAnimations.DISABLED);

        new Trigger(DriverStation::isFMSAttached)
                .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
                .onTrue(new InstantCommand(() -> this.setLed(LedAnimations._20S_LEFT, 2)));
    }

    
    private void sendAnimation(LedAnimations anim) {
        candle.animate(anim.getAnim());
    }

    public void setLed(LedAnimations anim, double seconds) {
        this.timer.restart();
        this.animation = anim;
        this.duration = seconds;
    }


    public void setLed(LedAnimations anim) {
        setLed(anim, Double.POSITIVE_INFINITY);
    }

    public void setDefault() {
        this.duration = 0.0;
        if (DriverStation.isDisabled()) {
            this.animation = LedAnimations.DISABLED;
        } else if (DriverStation.isAutonomous()) {
            this.animation = LedAnimations.AUTO;
        } else {
            this.animation = LedAnimations.TELEOP;
        }
        sendAnimation(this.animation);
    }

    public void run() {
        if (timer.hasElapsed(duration)) {
            setDefault();
        }
        if (animation != lastPattern || duration == 0) {
            sendAnimation(animation);
            lastPattern = animation;
        }
    }
}
