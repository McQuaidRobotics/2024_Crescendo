package com.igknighters.subsystems.swerve.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SPGCalcs {
    private static final double kEpsilon = 1E-8;
    private static final int MAX_STEER_ITERATIONS = 8;
    private static final int MAX_DRIVE_ITERATIONS = 10;

    static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    interface Function2d {
        double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
     * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
     * while being robust in ways that e.g. the Newton-Raphson method isn't.
     *
     * @param func The Function2d to take the root of.
     * @param x_0 x value of the lower bracket.
     * @param y_0 y value of the lower bracket.
     * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param x_1 x value of the upper bracket.
     * @param y_1 y value of the upper bracket.
     * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
     *     recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
     *     (approximate) root.
     */
    static double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left) {
        var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));

        if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
            return s_guess;
        }

        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess
                    + (1.0 - s_guess)
                            * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess
                    * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    static double findSteeringMaxS(
            double prevVx,
            double prevVy,
            double prevHeading,
            double desiredVx,
            double desiredVy,
            double desiredHeading,
            double maxDeviation) {
        desiredHeading = unwrapAngle(prevHeading, desiredHeading);
        double diff = desiredHeading - prevHeading;
        if (Math.abs(diff) <= maxDeviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = prevHeading + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> unwrapAngle(prevHeading, Math.atan2(y, x)) - offset;
        return findRoot(func, prevVx, prevVy, prevHeading - offset, desiredVx, desiredVy, desiredHeading - offset, MAX_STEER_ITERATIONS);
    }

    static double findDriveMaxS(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_vel_step
        ) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func = (x, y) -> Math.hypot(x, y) - offset;
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, MAX_DRIVE_ITERATIONS);
    }

    static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
        return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
                && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
                && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
     * direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e.
     *     prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive
     *     direction.
     */
    static boolean flipHeading(double prevToGoal) {
        return Math.abs(prevToGoal) > Math.PI / 2.0;
    }
}
