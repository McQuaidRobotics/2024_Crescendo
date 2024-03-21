package com.igknighters;

import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.igknighters.constants.ConstValues;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import monologue.MonoDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LED {
    private static class NoLED extends LED {
        NoLED() {
            super(false);
        }

        @Override
        public void runInner() {
            // Do nothing
        }

        @Override
        public DurationHandle sendMultiAnimationInner(double[] percents, LedAnimations[] anims) {
            return new DurationHandle(this);
        }
    }

    private final static int NUMLEDS = 38;
    private final static int CANDLE_LEDS = 8;

    private static LED instance;

    private static LED getInstance() {
        if (instance == null) {
            if (ConstValues.LED_ENABLED && !Robot.isSimulation()) {
                instance = new LED(true);
            } else {
                instance = new NoLED();
            }
        }
        return instance;
    }

    protected final CANdle candle;

    private final Timer timer = new Timer();
    private double duration = 0;

    private final ArrayList<PartialAnimation> animations = new ArrayList<>();

    private int robotMode = 0;
    private int lastMode = 0;

    /**
     * Defines specific behavior for an animation
     */
    public static class LEDAnimDescriptor {
        public int r = 0, g = 0, b = 0;
        public double speed;
        public Direction direction;
        public double brightness;

        /**
         * Descriptor used for generating animations
         * 
         * @param r         The red value [0, 255]
         * @param g         The green value [0, 255]
         * @param b         The blue value [0, 255]
         * @param speed     The speed of the animation. Effective [0.1, 1.0]
         * @param direction The direction of the animation. Either forward or backward
         */
        public LEDAnimDescriptor(int r, int g, int b, double speed, Direction direction) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.speed = speed;
            this.direction = direction;
        }

        /**
         * Descriptor used for generating rainbow animations
         * 
         * @param brightness Brightness of LEDs [0, 1]
         * @param speed      The speed of the animation. Effective [0.1, 1]
         */
        public LEDAnimDescriptor(double brightness, double speed) {
            this.brightness = brightness;
            this.speed = speed;
        }

        /**
         * Generates an animation to be passed into the CANdle
         * 
         * @param fn        Values used for animation generation other than the number
         *                  of LEDs and the offset.
         * @param numLed
         * @param ledOffset
         * @return Newly generated Animation object
         */
        public Animation generateAnimation(TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> fn, int numLed,
                int ledOffset) {
            return fn.apply(this, numLed, ledOffset);
        }
    }

    public enum LedAnimations {
        DISABLED(
                new LEDAnimDescriptor(255, 0, 0, 0.5, Direction.Forward),
                (desc, num, offset) -> {
                    return new ColorFlowAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset);
                }),
        TELEOP(new LEDAnimDescriptor(0, 255, 0, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new ColorFlowAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset);
                }),
        AUTO(new LEDAnimDescriptor(1.0, 0.5),
                (desc, num, offset) -> {
                    return new RainbowAnimation(
                            desc.brightness, desc.speed, num, false, offset);
                }),
        TEST(new LEDAnimDescriptor(0, 0, 255, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new ColorFlowAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset);
                }),
        _20S_LEFT(new LEDAnimDescriptor(255, 0, 255, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new StrobeAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, offset);
                }),
        SHOOTING(new LEDAnimDescriptor(15, 165, 165, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new ColorFlowAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, desc.direction, offset);
                }),
        WARNING(new LEDAnimDescriptor(252, 169, 15, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new StrobeAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, offset);
                }),
        ERROR(new LEDAnimDescriptor(255, 0, 0, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new StrobeAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, offset);
                }),
        INTAKE(new LEDAnimDescriptor(255, 255, 255, 0.2, Direction.Forward),
                (desc, num, offset) -> {
                    return new StrobeAnimation(
                            desc.r, desc.g, desc.b, 0, desc.speed, num, offset);
                }),
        OFF(new LEDAnimDescriptor(0, 0), (desc, num, offset) -> {
            return new RainbowAnimation(
                    desc.brightness, desc.speed, num);
        });

        private final LEDAnimDescriptor desc;
        private final TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> genFn;

        /**
         * Gets the animation defined by the enumeration
         * 
         * @param numLed    The number of LEDs to use for the animation [0,
         *                  <code>NUMLEDS</code>]
         * @param ledOffset The number of LEDs to offset the animation by [0,
         *                  <code>NUMLEDS</code>]
         * @return Newly generated animation as defined by <code>generateAnim()</code>
         */
        public Animation getAnimation(int numLed, int ledOffset) {
            return desc.generateAnimation(genFn, numLed, ledOffset);
        }

        private LedAnimations(LEDAnimDescriptor desc, TriFunction<LEDAnimDescriptor, Integer, Integer, Animation> fn) {
            this.desc = desc;
            this.genFn = fn;
        }
    }

    /**
     * Creates a partial animation which has a number of LEDs, an offset, and an
     * animation to animate on the CANdle
     * 
     * @param leds   The number of LEDs to animate on
     * @param offset The number of LEDs to offset the animation by
     * @param anim   The animation to pass through the CANdle
     */
    private static record PartialAnimation(
            int leds,
            int offset,
            LedAnimations anim) {
        public Animation getAnim() {
            return this.anim.getAnimation(leds, offset);
        }

        @Override
        public String toString() {
            return String.format(
                    "PartialAnimation(%s, %s, %s)",
                    this.leds, this.offset, this.anim.name());
        }
    }

    /**
     * Handles the duration that animations should run for.
     */
    public class DurationHandle {
        LED led;

        private DurationHandle(LED led) {
            this.led = led;
        }

        public void withDuration(double duration) {
            led.duration = duration;
        }
    }

    public LED(boolean makeLed) {
        timer.start();
        if (makeLed) {
            candle = new CANdle(52);
            var config = new CANdleConfiguration();
            config.v5Enabled = true;
            config.stripType = LEDStripType.RGB;
            config.brightnessScalar = 1.0;
            config.disableWhenLOS = true;
            candle.configAllSettings(config);

            new Trigger(DriverStation::isFMSAttached)
                    .and(() -> Math.abs(DriverStation.getMatchTime() - 20.0) < 0.2)
                    .onTrue(new InstantCommand(() -> LED.sendAnimation(LedAnimations._20S_LEFT)
                            .withDuration(2.0)).withName("20sLeftLED"));
        } else {
            candle = null;
        }
    }

    /**
     * Sends an animation to be animated on the CANdle on 100% of the strip
     * 
     * @param anim The animation to be animated
     * @return A <code>DurationHandle</code>
     */
    public static DurationHandle sendAnimation(LedAnimations anim) {
        return sendMultiAnimation(1.0, anim);
    }

    public static DurationHandle sendMultiAnimation(
            double percent1, LedAnimations anim1) {
        return getInstance().sendMultiAnimationInner(
                new double[] { percent1 },
                new LedAnimations[] { anim1 });
    }

    public static DurationHandle sendMultiAnimation(
            double percent1, LedAnimations anim1,
            double percent2, LedAnimations anim2) {
        return getInstance().sendMultiAnimationInner(
                new double[] { percent1, percent2 },
                new LedAnimations[] { anim1, anim2 });
    }

    public static DurationHandle sendMultiAnimation(
            double percent1, LedAnimations anim1,
            double percent2, LedAnimations anim2,
            double percent3, LedAnimations anim3) {
        return getInstance().sendMultiAnimationInner(
                new double[] { percent1, percent2, percent3 },
                new LedAnimations[] { anim1, anim2, anim3 });
    }

    public static DurationHandle sendMultiAnimation(
            double percent1, LedAnimations anim1,
            double percent2, LedAnimations anim2,
            double percent3, LedAnimations anim3,
            double percent4, LedAnimations anim4) {
        return getInstance().sendMultiAnimationInner(
                new double[] { percent1, percent2, percent3, percent4 },
                new LedAnimations[] { anim1, anim2, anim3, anim4 });
    }

    /**
     * Sends animation(s) to be animated by the CANdle on a certain percent of the
     * strip. Handles up to 5 animations and 5 percents.
     * 
     * @param percent1 The first percent of the LEDs to be used for an animation.
     *                 Pairs with <code>anim1</code>
     * @param anim1    The first animation to be animated on the CANdle. Pairs with
     *                 <code>percent1</code>
     * @param percent2 The second percent of the LEDs to be used for an animation.
     *                 Pairs with <code>anim2</code>
     * @param anim2    The second animation to be animated on the CANdle. Pairs with
     *                 <code>percent2</code>
     * @param percent3 The third percent of the LEDs to be used for an animation.
     *                 Pairs with <code>anim3</code>
     * @param anim3    The third animation to be animated on the CANdle. Pairs with
     *                 <code>percent3</code>
     * @param percent4 The fourth percent of the LEDs to be used for an animation.
     *                 Pairs with <code>anim4</code>
     * @param anim4    The fourth animation to be animated on the CANdle. Pairs with
     *                 <code>percent4</code>
     * @param percent5 The fifth percent of the LEDs to be used for an animation.
     *                 Pairs with <code>anim5</code>
     * @param anim5    The fifth animation to be animated on the CANdle. Pairs with
     *                 <code>percent5</code>
     * @return A {@code DurationHandle}
     */
    public static DurationHandle sendMultiAnimation(
            double percent1, LedAnimations anim1,
            double percent2, LedAnimations anim2,
            double percent3, LedAnimations anim3,
            double percent4, LedAnimations anim4,
            double percent5, LedAnimations anim5) {
        return getInstance().sendMultiAnimationInner(
                new double[] { percent1, percent2, percent3, percent4, percent5 },
                new LedAnimations[] { anim1, anim2, anim3, anim4, anim5 });
    }

    /**
     * Creates new partial animations defined by {@code sendMultiAnimation}
     * 
     * If {@code percents} and {@code anims} are not the same length,
     * or if the length is greater than 10, an error is reported to the
     * DriverStation.
     * These warnings can cause loss of sent animations, if expected behavior is not
     * happening check for these warnings.
     * 
     * @param percents The percents for each animation to be applied to
     * @param anims    The animations to be passed through the CANdle
     * @return A {@code DurationHandle}
     */
    public DurationHandle sendMultiAnimationInner(
            final double[] percents, final LedAnimations[] anims) {
        if (percents.length != anims.length || anims.length > 9) {
            DriverStation.reportError(
                    "Send Multi Animation Error: AnimsLength(" + anims.length
                            + ") PercentsLength(" + percents.length + ")",
                    true);
        }

        int count = Math.min(percents.length, anims.length);

        this.animations.clear();

        int offset = 0;
        double used = 0.0;

        for (int i = 0; i < count; i++) {
            double percent = percents[i];
            LedAnimations anim = anims[i];

            if (1.0 - used < percent) {
                percent = 1.0 - used;
            }

            used += percent;

            int numPixels = (int) Math.floor(NUMLEDS * percent);

            this.animations.add(new PartialAnimation(numPixels, offset + CANDLE_LEDS, anim));

            offset += numPixels;

            if (offset == NUMLEDS) {
                break;
            }
        }

        this.duration = 9999.0;
        this.timer.restart();
        return new DurationHandle(this);
    }

    /**
     * Sends an animation to be animated based on the state of the robot
     * {@code [DISABLED, AUTO, TELEOP]}.
     * Also sets the {@code robotMode} variable to a value based on the mode of the
     * robot to track the most recent mode.
     * Also sets the {@code duration} to 0.0
     */
    public void setDefault() {
        if (DriverStation.isDisabled()) {
            sendAnimation(LedAnimations.DISABLED);
            robotMode = 0;
        } else if (DriverStation.isAutonomous()) {
            sendAnimation(LedAnimations.AUTO);
            robotMode = 1;
        } else {
            sendAnimation(LedAnimations.TELEOP);
            robotMode = 2;
        }
        this.duration = 0.0;
    }

    public static void run() {
        getInstance().runInner();
    }

    /**
     * Runs the {@code LED} code, passing animations through the CANdle. Clears
     * current animation if {@code robotMode != lastMode}.
     * Logs the current pattern that is expected to be passed through the CANdle
     */
    public void runInner() {
        if (timer.hasElapsed(duration)) {
            setDefault();
        }
        String log = "[";
        for (int i = 0; i < this.animations.size(); i++) {
            PartialAnimation partial = this.animations.get(i);
            if (lastMode != robotMode) {
                candle.animate(null);
            }
            candle.animate(partial.getAnim(), i + 1);

            log += partial.toString() + ",";
            lastMode = robotMode;
        }
        MonoDashboard.put("LED", log + "]");

        double batteryVolts = RobotController.getBatteryVoltage();
        if (batteryVolts < RobotController.getBrownoutVoltage() || RobotController.isBrownedOut()) {
            candle.animate(new StrobeAnimation(255, 0, 0, 255, 0.5, CANDLE_LEDS), 0);
        } else {
            // the lower the battery the more red the pattern
            double percent = MathUtil.clamp(batteryVolts - 8.0, 0.0, 4.5) / 4.5;
            int green = (int) (percent * 255);
            int red = (int) ((1 - percent) * 255);

            candle.setLEDs(red, green, 0, 255, 0, CANDLE_LEDS);
        }
    }
}
