package igknighters.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import igknighters.Localizer;
import igknighters.SimCtx;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.stem.Stem;

import igknighters.subsystems.swerve.Swerve;

import igknighters.subsystems.umbrella.Umbrella;

import igknighters.subsystems.vision.Vision;
import igknighters.util.logging.BootupLogger;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import monologue.Logged;
import monologue.ProceduralStructGenerator;
import monologue.Annotations.Log;
import monologue.Annotations.OptionalLogged;
import monologue.ProceduralStructGenerator.IgnoreStructField;

public class SubsystemResources {

    /**
     * a way to pass around data about enabled subsystems
     */
    public enum Subsystems implements StructSerializable {
        Stem("Stem"),
        Swerve("Swerve"),
        Umbrella("Umbrella"),
        Vision("Vision"),
        Led("Led");

        @IgnoreStructField
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

        public static final Struct<Subsystems> struct = ProceduralStructGenerator.genEnum(Subsystems.class);
    }

    public static class AllSubsystems implements Logged {
        private List<LockFullSubsystem> subsystemsListLockFull = new ArrayList<>();
        private List<LockFreeSubsystem> subsystemsListLockFree = new ArrayList<>();
        private Subsystems[] subsystems;

        @Log(key = "Stem")
        @OptionalLogged(type = Stem.class)
        public final Optional<Stem> stem;

        @Log(key = "Swerve")
        @OptionalLogged(type = Swerve.class)
        public final Optional<Swerve> swerve;

        @Log(key = "Umbrella")
        @OptionalLogged(type = Umbrella.class)
        public final Optional<Umbrella> umbrella;

        @Log(key = "Vision")
        @OptionalLogged(type = Vision.class)
        public final Optional<Vision> vision;

        @Log(key = "Led")
        @OptionalLogged(type = Led.class)
        public final Optional<Led> led;

        public AllSubsystems(Localizer localizer, SimCtx simCtx, Subsystems... subsystems) {
            this.subsystems = subsystems;

            if (subsystems.length == 0) {
                BootupLogger.bootupLog("No subsystems enabled");
            }

            List<Subsystems> enabledSubsystems = List.of(subsystems);

            if (enabledSubsystems.contains(Subsystems.Stem)) {
                stem = createSubsystem(Stem::new, simCtx);
            } else {
                stem = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Swerve)) {
                swerve = createSubsystem(Swerve::new, localizer, simCtx);
            } else {
                swerve = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Umbrella)) {
                umbrella = createSubsystem(Umbrella::new, simCtx);
            } else {
                umbrella = Optional.empty();
            }

            if (enabledSubsystems.contains(Subsystems.Vision)) {
                vision = createSubsystem(Vision::new, localizer, simCtx);
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

        private <T extends Object> Optional<T> createSubsystem(Function<SimCtx, T> subsystemSupplier, SimCtx simRobot) {
            T subsystem = subsystemSupplier.apply(simRobot);
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

        private <T extends Object> Optional<T> createSubsystem(BiFunction<Localizer, SimCtx, T> subsystemSupplier, Localizer localizer, SimCtx simRobot) {
            T subsystem = subsystemSupplier.apply(localizer, simRobot);
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

    /**
     * @see Subsystem
     */
    public static interface LockFullSubsystem extends Subsystem, Logged {
    }

    /**
     * A subsystem that does not act as a locking resource
     * when using commands
     */
    public static interface LockFreeSubsystem extends Logged {
        default void periodic() {}
        default void simulationPeriodic() {}

        default String getName() {
            return this.getClass().getSimpleName();
        }
    }
}
