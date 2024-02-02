package com.igknighters;

import java.util.HashMap;
import java.util.function.Supplier;

import javax.naming.ldap.HasControls;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

    public static class LEDAnimDescriptor {
        public int r, g, b;
        public double speed;
        public Direction direction;
        public double brightness;

        public LEDAnimDescriptor(int r, int g, int b, double speed, Direction direction) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.speed = speed;
            this.direction = direction;
        }

        public LEDAnimDescriptor(double brightness, double speed) {
            this.brightness = brightness;
            this.speed = speed;
        }

        public LEDAnimDescriptor() {}

        public Animation generateAnim(TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> fn, int numLed, int ledOffset) {
            return fn.apply(this, numLed, ledOffset);
        }
    }

    private final static int NUMLEDS = 8;
    private static int numPatterns = 1;
    
    public enum LedAnimations {
        DISABLED(
            new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }
        ),
        TELEOP(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        AUTO(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new RainbowAnimation(
                    desc.brightness, desc.speed, num
                );
            }),
        TEST(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        _20S_LEFT(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new SingleFadeAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, offset
                );
            }),
        SHOOTING(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        BOOTING(new LEDAnimDescriptor(),
            (desc, num, offset) -> {
                return new StrobeAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, offset
                );
            });

        private final LEDAnimDescriptor desc;
        private final TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> genFn;

        public Animation getAnim(int numLed, int ledOffset){
            return desc.generateAnim(genFn, numLed, ledOffset);
        }
        
        private LedAnimations(LEDAnimDescriptor desc, TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> fn) {
            this.desc = desc;
            this.genFn = fn;
        }
    }
    
    
    
    private Timer timer;
    private Double duration;
    private LedAnimations animation;
    private LedAnimations animation2;
    private LedAnimations lastAnimation;
    private CANdle candle;
    private CANdleConfiguration config;
    private XboxController controller = new XboxController(0);

    public LED() {
        this.timer = new Timer();
        // this.animation = LedAnimations.DISABLED;
        // this.animation2 = LedAnimations.DEFAULT;
        // this.lastPattern = LedAnimations.DISABLED;
        this.duration = 0.0;
        candle = new CANdle(51);
        config = new CANdleConfiguration();
        config.v5Enabled = true;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        config.disableWhenLOS = true;
        candle.configAllSettings(config);
        sendAnimation(animation1);

    }


    //     new Trigger(DriverStation::isFMSAttached)
    //             .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
    //             .onTrue(new InstantCommand(() -> this.setLed(LedAnimations._20S_LEFT, 2)));

    // }

    
    // private void sendAnimation(LedAnimations anim) {
    //     candle.animate(anim.getAnim());
    // }
    
    private void sendAnimation(Animation anim){
        candle.animate(anim);
    }
    
    // private void sendMultiAnimation(LedAnimations anim1, LedAnimations anim2){
    //     candle.animate(anim1.getAnim(), 0);
    //     candle.animate(anim2.getAnim(), 1);
    // }
    private void sendMultiAnimation(Animation anim1, Animation anim2){
        candle.animate(anim1, 0);
        candle.animate(anim2, 1);
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
        numPatterns = 1;
        this.duration = 0.0;
        if (DriverStation.isDisabled()) {
            this.animation = LedAnimations.DISABLED;
        } else if (DriverStation.isAutonomous()) {
            this.animation = LedAnimations.AUTO;
        } else {
            this.animation = LedAnimations.TELEOP;
        }
        
    }

    public void shooting(Animation anim) {
        if (timer.hasElapsed(duration)){
            // this.animation = LedAnimations.NONE;
        }
        else{
            this.animation2 = LedAnimations.SHOOTING;
            sendMultiAnimation(animation, animation2);
        }
    }

    public void run() {
        if (timer.hasElapsed(duration)) {
            setDefault();
            animation2 = LedAnimations.NONE;
        }
        if (animation != lastAnimation || duration == 0) {
            //sendMultiAnimation(animation, animation2);
            sendMultiAnimation(animation, animation2);
            lastAnimation = animation;
        }

    }
    

}
