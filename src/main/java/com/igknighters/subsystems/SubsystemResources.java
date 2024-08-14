package com.igknighters.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.igknighters.subsystems.stem.Stem;

import com.igknighters.subsystems.swerve.Swerve;

import com.igknighters.subsystems.umbrella.Umbrella;

import com.igknighters.subsystems.vision.Vision;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SubsystemResources {

    /**
     * a way to pass around data about enabled subsystems
     */
    public enum Subsystems {

        Stem("Stem"),

        Swerve("Swerve"),

        Umbrella("Umbrella"),

        Vision("Vision"),

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
        private List<Subsystem> subsystemsList = new ArrayList<>();

        public final Optional<Stem> stem;

        public final Optional<Swerve> swerve;

        public final Optional<Umbrella> umbrella;

        public final Optional<Vision> vision;

        public AllSubsystems(Subsystems... subsystems) {
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
                swerve = createSubsystem(Swerve::new);
            } else {
                swerve = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Umbrella)) {
                umbrella = createSubsystem(Umbrella::new);
            } else {
                umbrella = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Vision)) {
                vision = createSubsystem(Vision::new);
            } else {
                vision = Optional.empty();
            }
        }

        private <T extends Subsystem> Optional<T> createSubsystem(Supplier<T> subsystemSupplier) {
            T subsystem = subsystemSupplier.get();
            BootupLogger.bootupLog("Subsystem " + subsystem.getClass().getSimpleName() + " created");
            subsystemsList.add(subsystem);
            return Optional.of(subsystem);
        }

        public Subsystems[] getEnabledSubsystemEnums() {
            return subsystems;
        }

        public List<Subsystem> getEnabledSubsystems() {
            return subsystemsList;
        }

        public Subsystem[] getEnabledSubsystemsArr() {
            return subsystemsList.toArray(new Subsystem[subsystemsList.size()]);
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
}
