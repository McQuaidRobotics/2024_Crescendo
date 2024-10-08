package com.igknighters.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import com.igknighters.Localizer;
import com.igknighters.subsystems.led.Led;
import com.igknighters.subsystems.stem.Stem;

import com.igknighters.subsystems.swerve.Swerve;

import com.igknighters.subsystems.umbrella.Umbrella;

import com.igknighters.subsystems.vision.Vision;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;

public class SubsystemResources {

    /**
     * a way to pass around data about enabled subsystems
     */
    public enum Subsystems {

        Stem("Stem"),

        Swerve("Swerve"),

        Umbrella("Umbrella"),

        Vision("Vision"),

        Led("Led"),

        ;

        public final String name;

        Subsystems(String name) {
            this.name = name;
        }

        /**
         * a prettier way to make an array of subsystems
         * 
         * @param subsystems
         * @return an array of subsystems
         */
        public static Subsystems[] list(Subsystems... subsystems) {
            return subsystems;
        }

        /**
         * a prettier way to make an array of subsystems
         * 
         * @param subsystems
         * @return an array of subsystems
         */
        public static Subsystems[] list(String... subsystems) {
            for (String subsystem : subsystems) {
                if (!subsystemExists(subsystem)) {
                    throw new IllegalArgumentException("Subsystem " + subsystem + " does not exist");
                }
            }
            Subsystems[] subsystemsArray = new Subsystems[subsystems.length];
            for (int i = 0; i < subsystems.length; i++) {
                subsystemsArray[i] = Subsystems.valueOf(subsystems[i]);
            }
            return subsystemsArray;
        }

        public static Subsystems[] all() {
            return Subsystems.values();
        }

        public static Subsystems[] none() {
            return new Subsystems[] {};
        }

        public static boolean subsystemExists(String subsystem) {
            for (Subsystems subsystem1 : Subsystems.values()) {
                if (subsystem1.name.equals(subsystem)) {
                    return true;
                }
            }
            return false;
        }
    }

    public static class AllSubsystems {
        private Subsystems[] subsystems;
        private List<LockFullSubsystem> subsystemsListLockFull = new ArrayList<>();
        private List<LockFreeSubsystem> subsystemsListLockFree = new ArrayList<>();

        public final Optional<Stem> stem;

        public final Optional<Swerve> swerve;

        public final Optional<Umbrella> umbrella;

        public final Optional<Vision> vision;

        public final Optional<Led> led;

        public AllSubsystems(Localizer localizer, Subsystems... subsystems) {
            this.subsystems = subsystems;

            if (subsystems.length == 0) {
                BootupLogger.bootupLog("No subsystems enabled");
            }

            List<Subsystems> enabledSubsystems = List.of(subsystems);

            if (enabledSubsystems.contains(Subsystems.Stem)) {
                stem = createSubsystem(Stem::new);
            } else {
                stem = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Swerve)) {
                swerve = createSubsystem(Swerve::new, localizer);
            } else {
                swerve = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Umbrella)) {
                umbrella = createSubsystem(Umbrella::new);
            } else {
                umbrella = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Vision)) {
                vision = createSubsystem(Vision::new, localizer);
            } else {
                vision = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Led)) {
                led = createSubsystem(Led::new);
            } else {
                led = Optional.empty();
            }

            CommandScheduler.getInstance().registerSubsystem(
                getEnabledLockFullSubsystemsArr()
            );
            for (LockFreeSubsystem subsystem : getEnabledLockFreeSubsystemsArr()) {
                CommandScheduler.getInstance().registerSubsystem(
                    new Subsystem() {
                        @Override
                        public void periodic() {
                            subsystem.periodic();
                        }

                        @Override
                        public String getName() {
                            return subsystem.getName();
                        }
                    }
                );
            }
        }

        private <T extends Object> Optional<T> createSubsystem(Supplier<T> subsystemSupplier) {
            T subsystem = subsystemSupplier.get();
            BootupLogger.bootupLog("Subsystem " + subsystem.getClass().getSimpleName() + " created");
            if (subsystem instanceof LockFullSubsystem) {
                subsystemsListLockFull.add((LockFullSubsystem) subsystem);
            } else if (subsystem instanceof LockFreeSubsystem) {
                subsystemsListLockFree.add((LockFreeSubsystem) subsystem);
            } else {
                throw new IllegalArgumentException("Subsystem " + subsystem.getClass().getSimpleName() + " is not a valid subsystem");
            }
            return Optional.of(subsystem);
        }

        private <T extends Object> Optional<T> createSubsystem(Function<Localizer, T> subsystemSupplier, Localizer localizer) {
            T subsystem = subsystemSupplier.apply(localizer);
            BootupLogger.bootupLog("Subsystem " + subsystem.getClass().getSimpleName() + " created");
            if (subsystem instanceof LockFullSubsystem) {
                subsystemsListLockFull.add((LockFullSubsystem) subsystem);
            } else if (subsystem instanceof LockFreeSubsystem) {
                subsystemsListLockFree.add((LockFreeSubsystem) subsystem);
            } else {
                throw new IllegalArgumentException("Subsystem " + subsystem.getClass().getSimpleName() + " is not a valid subsystem");
            }
            return Optional.of(subsystem);
        }

        public Subsystems[] getEnabledSubsystemEnums() {
            return subsystems;
        }

        public LockFullSubsystem[] getEnabledLockFullSubsystemsArr() {
            return subsystemsListLockFull.toArray(new LockFullSubsystem[subsystemsListLockFull.size()]);
        }

        public LockFreeSubsystem[] getEnabledLockFreeSubsystemsArr() {
            return subsystemsListLockFree.toArray(new LockFreeSubsystem[subsystemsListLockFree.size()]);
        }

        public List<Logged> getLoggableSubsystems() {
            List<Logged> loggableSubsystems = new ArrayList<>();
            loggableSubsystems.addAll(subsystemsListLockFull);
            loggableSubsystems.addAll(subsystemsListLockFree);
            return loggableSubsystems;
        }

        public boolean hasAllSubsystems() {

            if (!stem.isPresent()) {
                return false;
            }

            if (!swerve.isPresent()) {
                return false;
            }

            if (!umbrella.isPresent()) {
                return false;
            }

            if (!vision.isPresent()) {
                return false;
            }

            return true;
        }
    }

    public static interface LockFullSubsystem extends Subsystem, Logged {
        @Override
        default String getOverrideName() {
            return this.getName();
        }
    }

    public static interface LockFreeSubsystem extends Logged {
        default void periodic() {}
        default void simulationPeriodic() {}

        default String getName() {
            return this.getClass().getSimpleName();
        }

        @Override
        default String getOverrideName() {
            return this.getName();
        }
    }
}
