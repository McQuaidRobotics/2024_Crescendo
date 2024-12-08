package sham.utils;

/**
 * A type safe way of representing gear ratios
 * keeping the context of the gear ratio in mind.
 */
public sealed interface GearRatio {

    /**
     * Get the reduction of the gear ratio.
     * 
     * @return the reduction of the gear ratio
     */
    double getReduction();

    /**
     * Get the overdrive of the gear ratio.
     * 
     * @return the overdrive of the gear ratio
     */
    default double getOverdrive() {
        return 1.0 / getReduction();
    }

    /**
     * Combine this gear ratio with another gear ratio.
     * 
     * @param next the next gear ratio to combine with
     * @return the combined gear ratio
     */
    default GearRatio then(GearRatio next) {
        return new Reduction(getReduction() * next.getReduction());
    }

    /**
     * Combine this gear ratio with a reduction.
     * 
     * @param reduction the reduction to combine with
     * @return the combined gear ratio
     */
    default GearRatio thenReduction(double reduction) {
        return new Reduction(getReduction() * reduction);
    }

    /**
     * Combine this gear ratio with an overdrive.
     * 
     * @param overdrive the overdrive to combine with
     * @return the combined gear ratio
     */
    default GearRatio thenOverdrive(double overdrive) {
        return new Overdrive(getOverdrive() * overdrive);
    }

    /**
     * A type safe representation of a gearing reduction.
     */
    public record Reduction(double reduction) implements GearRatio {
        @Override
        public double getReduction() {
            return reduction;
        }
    }

    /**
     * A type safe representation of a gearing overdrive.
     */
    public record Overdrive(double overdrive) implements GearRatio {
        @Override
        public double getReduction() {
            return 1.0 / overdrive;
        }
    }

    /**
     * Create a new gear ratio with the given reduction.
     * 
     * <p> Reduction means a number above 1.0 causes the output to be slower than the input.
     * 
     * @param reduction the reduction of the gear ratio
     * @return a new instance of {@link Reduction}
     */
    public static Reduction reduction(double reduction) {
        return new Reduction(reduction);
    }

    /**
     * Create a new gear ratio with the given overdrive.
     * 
     * <p> Overdrive means a number above 1.0 causes the output to be faster than the input.
     * 
     * @param overdrive the overdrive of the gear ratio
     * @return a new instance of {@link Overdrive}
     */
    public static Overdrive overdrive(double overdrive) {
        return new Overdrive(overdrive);
    }
}
