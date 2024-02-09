package com.igknighters;

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
        public int r = 0, g = 0, b = 0;
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

        public Animation generateAnim(TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> fn, int numLed, int ledOffset) {
            return fn.apply(this, numLed, ledOffset);
        }
    }

    private final static int NUMLEDS = 8;
    private static int numPatterns = 1;
    
    public enum LedAnimations {
        DISABLED(
            new LEDAnimDescriptor(255, 0, 0, 0.2, Direction.Forward),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }
        ),
        TELEOP(new LEDAnimDescriptor(0, 255, 0, 0.2, Direction.Forward),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        AUTO(new LEDAnimDescriptor(1.0, 0.5),
            (desc, num, offset) -> {
                return new RainbowAnimation(
                    desc.brightness, desc.speed, num
                );
            }),
        TEST(new LEDAnimDescriptor(0, 0, 255, 0.2, Direction.Forward),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        _20S_LEFT(new LEDAnimDescriptor(255, 0, 255, 0.2, Direction.Forward),
            (desc, num, offset) -> {
                return new SingleFadeAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, offset
                );
            }),
        SHOOTING(new LEDAnimDescriptor(245, 174, 10, 0.2, Direction.Forward),
            (desc, num, offset) -> {
                return new ColorFlowAnimation(
                    desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset
                );
            }),
        BOOTING(new LEDAnimDescriptor(255, 255, 255, 0.2, Direction.Forward),
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
    private LedAnimations animation1;
    private LedAnimations animation2;
    private LedAnimations lastAnimation;
    private CANdle candle;
    private CANdleConfiguration config;
    private XboxController controller = new XboxController(0);

    public LED() {
        this.timer = new Timer();
        this.animation1 = LedAnimations.DISABLED;
        this.animation2 = LedAnimations.DISABLED;
        this.lastAnimation = LedAnimations.DISABLED;
        this.duration = 0.0;
        candle = new CANdle(52);
        config = new CANdleConfiguration();
        config.v5Enabled = true;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        config.disableWhenLOS = true;
        candle.configAllSettings(config);
        sendAnimation(animation1, 0);

        new Trigger(DriverStation::isFMSAttached)
                .and(() -> Math.abs(DriverStation.getMatchTime() - 30.0) < 0.2)
                .onTrue(new InstantCommand(() -> this.setLed(LedAnimations._20S_LEFT, 2)));

    }

    
    private void sendAnimation(LedAnimations anim, int offset) {
        candle.animate(anim.getAnim(NUMLEDS/numPatterns, offset));
    }
    
    private void sendMultiAnimation(LedAnimations anim1, LedAnimations anim2){
        candle.animate(anim1.getAnim(NUMLEDS/numPatterns, 0), 0);
        candle.animate(anim2.getAnim(NUMLEDS/numPatterns, NUMLEDS/numPatterns), 1);
    }

    public void setLed(LedAnimations anim, double seconds) {
        this.timer.restart();
        this.animation1 = anim;
        this.duration = seconds;
    }


    public void setLed(LedAnimations anim) {
        setLed(anim, Double.POSITIVE_INFINITY);
    }

    public void setDefault() {
        numPatterns = 1;
        animation2 = LedAnimations.DISABLED;
        this.duration = 0.0;
        if (DriverStation.isDisabled()) {
            this.animation1 = LedAnimations.DISABLED;
        } else if (DriverStation.isAutonomous()) {
            this.animation1 = LedAnimations.AUTO;
        } else {
            this.animation1 = LedAnimations.TELEOP;
        }
        
    }

    public void shooting() {
        duration = 3.0;
        numPatterns++;
        if (timer.hasElapsed(duration)){
            this.animation2 = LedAnimations.DISABLED;
        }
        else{
            if (DriverStation.isDisabled()) {
            this.animation1 = LedAnimations.DISABLED;
        } else if (DriverStation.isAutonomous()) {
            this.animation1 = LedAnimations.AUTO;
        } else {
            this.animation1 = LedAnimations.TELEOP;
        }
            this.animation2 = LedAnimations.SHOOTING;
            sendMultiAnimation(animation1, animation2);
        }
    }

    public void run() {
        if (controller.getAButton()) {
            shooting();
        }
        if (timer.hasElapsed(duration)) {
            setDefault();
        }
        if (animation1 != lastAnimation || duration == 0) {
            sendMultiAnimation(animation1, animation2);
            lastAnimation = animation1;
        }

    }
    

}
