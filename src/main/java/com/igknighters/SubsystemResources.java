package com.igknighters;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.igknighters.subsystems.stem.Stem;

import com.igknighters.subsystems.swerve.Swerve;

import com.igknighters.subsystems.umbrella.Umbrella;

import com.igknighters.subsystems.vision.Vision;

import com.igknighters.util.BootupLogger;

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

        public Optional<Stem> stem = Optional.empty();

        public Optional<Swerve> swerve = Optional.empty();

        public Optional<Umbrella> umbrella = Optional.empty();

        public Optional<Vision> vision = Optional.empty();

        public AllSubsystems(Subsystems... subsystems) {
            this.subsystems = subsystems;
            for (Subsystems subsystem : subsystems) {
                switch (subsystem) {

                    case Stem:
                        stem = createSubsystem(Stem::new);
                        break;

                    case Swerve:
                        swerve = createSubsystem(Swerve::new);
                        break;

                    case Umbrella:
                        umbrella = createSubsystem(Umbrella::new);
                        break;

                    case Vision:
                        vision = createSubsystem(Vision::new);
                        break;

                    default:
                        break;
                }
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
