package com.igknighters.subsystems.led;

import com.igknighters.util.logging.ProceduralStructGenerator;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public enum LedAnimations {
    DISABLED(new LedPattern.Flow(255, 0, 0, 0, 0.5, false)),
    TELEOP(new LedPattern.Flow(0, 255, 0, 0, 0.2, false)),
    AUTO(new LedPattern.Rainbow(1.0, 0.5, false)),
    Test(new LedPattern.Flow(0, 0, 255, 0, 0.2, false)),
    _20S_LEFT(new LedPattern.Strobe(255, 0, 255, 100, 1.0)),
    SHOOTING(new LedPattern.Solid(15, 165, 165, 0)),
    WARNING(new LedPattern.Strobe(252, 169, 15, 30, 0.7)),
    Intake(new LedPattern.Solid(100, 0, 100, 100)),
    Off(new LedPattern.Solid(0, 0, 0, 0));

    public final LedPattern pattern;

    private LedAnimations(LedPattern pattern) {
        this.pattern = pattern;
    }

    public static final Struct<LedAnimations> struct = ProceduralStructGenerator.genEnum(LedAnimations.class);

    public sealed interface LedPattern extends StructSerializable {
        public record Solid(int r, int g, int b, int w) implements LedPattern {
            public static final Struct<Solid> struct = ProceduralStructGenerator.genRecord(Solid.class);
        }

        public record Strobe(int r, int g, int b, int w, double speed) implements LedPattern {
            public static final Struct<Strobe> struct = ProceduralStructGenerator.genRecord(Strobe.class);
        }

        public record Flow(int r, int g, int b, int w, double speed, boolean backward) implements LedPattern {
            public static final Struct<Flow> struct = ProceduralStructGenerator.genRecord(Flow.class);
        }

        public record Rainbow(double brightness, double speed, boolean backward) implements LedPattern {
            public static final Struct<Rainbow> struct = ProceduralStructGenerator.genRecord(Rainbow.class);
        }

        public record Fire(double brightness, double speed, double sparking, double cooling, boolean backward) implements LedPattern {
            public static final Struct<Fire> struct = ProceduralStructGenerator.genRecord(Fire.class);
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
    public static record PartialAnimation(
            int leds,
            int offset,
            LedAnimations anim) implements StructSerializable {

        public LedPattern getPattern() {
            return this.anim.pattern;
        }

        @Override
        public String toString() {
            return String.format(
                    "PartialAnimation(%s, %s, %s)",
                    this.leds, this.offset, this.anim.name());
        }

        public static final Struct<PartialAnimation> struct = ProceduralStructGenerator.genRecord(PartialAnimation.class);
    }
}
