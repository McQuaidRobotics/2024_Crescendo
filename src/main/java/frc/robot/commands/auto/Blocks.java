package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GamepieceMode;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.StateManager;
import frc.robot.commands.superstructure.StateManager.CmdTransitionState;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.super_structure.States;

public class Blocks {

    public static final HashMap<String, Command> EVENT_MAP = Cmds.getMap();

    public interface Block {
        public Command getCommand(SwerveAutoBuilder builder);

        public String getCommandName();
    }

    private static String ScreamingSnakeToCamal(String screamingSnake) {
        String[] words = screamingSnake.split("_");
        String camal = "";
        for (String word : words) {
            camal += word.substring(0, 1).toUpperCase() + word.substring(1).toLowerCase();
        }
        camal = camal.substring(0, 1).toLowerCase() + camal.substring(1);
        return camal;
    }

    public static enum Cmds implements Block {
        HOME(new CmdTransitionState(RobotContainer.superStructure, States.HOME)),
        HOME_TIMEOUT(new CmdTransitionState(RobotContainer.superStructure, States.HOME).withTimeout(0.5)),
        STOW(new CmdTransitionState(RobotContainer.superStructure, States.STOW)),

        PLACE_STANDBY(new CmdTransitionState(RobotContainer.superStructure, States.STANDBY).canFinish()),
        PLACE_HIGH(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_HIGH).canFinish()
                .andThen(StateManager.dispellGamepiece(RobotContainer.superStructure))),
        PLACE_MID(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_MID).canFinish()
                .andThen(StateManager.dispellGamepiece(RobotContainer.superStructure))),
        PLACE_LOW(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_LOW_FRONT).canFinish()
                .andThen(StateManager.dispellGamepiece(RobotContainer.superStructure))),

        PLACE_HIGH_NO_DROP(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_HIGH).canFinish()),
        PLACE_MID_NO_DROP(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_MID).canFinish()),
        PLACE_LOW_NO_DROP(new CmdTransitionState(RobotContainer.superStructure, States.PLACE_LOW_FRONT).canFinish()),

        PICKUP_GROUND(new CmdTransitionState(RobotContainer.superStructure, States.PICKUP_GROUND)),
        PICKUP_STATION(new CmdTransitionState(RobotContainer.superStructure, States.PICKUP_STATION)),

        DESIRE_CUBE(new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CUBE))),
        DESIRE_CONE(new InstantCommand(() -> GamepieceMode.setDesiredPiece(GamepieceMode.CONE))),

        OVERRIDE_HOLD_CUBE(new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CUBE))),
        OVERRIDE_HOLD_CONE(new InstantCommand(() -> GamepieceMode.setHeldPiece(GamepieceMode.CONE))),

        GYRO_0(new InstantCommand(() -> RobotContainer.swerve.setYaw(0.0))),
        GYRO_180(new InstantCommand(() -> RobotContainer.swerve.setYaw(180.0))),
        X_WHEELS(SwerveCommands.commandXDrives(RobotContainer.swerve));

        public final Command command;

        Cmds(Command command) {
            this.command = command;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return command;
        }

        @Override
        public String getCommandName() {
            return "Cmd: " + ScreamingSnakeToCamal(this.name());
        }

        private static HashMap<String, Command> getMap() {
            HashMap<String, Command> map = new HashMap<String, Command>();
            for (Cmds cmd : Cmds.values()) {
                map.put(ScreamingSnakeToCamal(cmd.name()), cmd.command);
            }
            return map;
        }
    }

    public static enum PPPaths implements Block {
        ONE_METER("1M_BI"),
        FLAT_BALANCE_SETUP("FLAT_BALANCE_SETUP"),
        FLAT_PICKUP3("FLAT_PICKUP3"),
        FLAT_PICKUP4("FLAT_PICKUP4"),
        FLAT_PLACE7("FLAT_PLACE7"),
        FLAT_PLACE8("FLAT_PLACE8"),
        FLAT_PLACE9("FLAT_PLACE9"),
        FLAT_SWOOP4("FLAT_SWOOP4"),
        FLAT_SWOOP4B("FLAT_SWOOP4B"),
        PICKUP1_WIRE("PICKUP1_WIRE"),
        PICKUP2_WIRE("PICKUP2_WIRE"),
        PICKUP3_FLAT("PICKUP3_FLAT"),
        PICKUP4_FLAT("PICKUP4_FLAT"),
        PLACE1_WIRE("PLACE1_WIRE"),
        PLACE2_WIRE("PLACE2_WIRE"),
        PLACE7_FLAT("PLACE7_FLAT"),
        PLACE8_FLAT("PLACE8_FLAT"),
        PLACE9_FLAT("PLACE9_FLAT"),
        WIRE_BALANCE_SETUP("WIRE_BALANCE_SETUP"),
        WIRE_OVER_IN("WIRE_OVER_IN"),
        WIRE_OVER_OUT("WIRE_OVER_OUT"),
        WIRE_PICKUP1("WIRE_PICKUP1"),
        WIRE_PICKUP2("WIRE_PICKUP2"),
        WIRE_PLACE1("WIRE_PLACE1"),
        WIRE_PLACE2("WIRE_PLACE2"),
        WIRE_PLACE3("WIRE_PLACE3"),
        PLACE_BAL("PLACE_BAL");

        public final String trajName;

        PPPaths(String trajName) {
            this.trajName = trajName;
        }

        public PathPlannerTrajectory getTraj() {
            Double maxSpeed = 4.0;
            if (trajName.contains("WIRE")) {
                maxSpeed = 2.25;
            }
            return PathLoader.openFilePath(trajName, maxSpeed);
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return builder.followPath(getTraj());
        }

        @Override
        public String getCommandName() {
            return "Path: " + ScreamingSnakeToCamal(this.name());
        }

        public CustomPath resetPose() {
            return new CustomPath(this::getTraj, true);
        }

        public CustomPath merge(Double percentThrough, Cmds block, Cmds... blocks) {
            return new CustomPath(this::getTraj, false, percentThrough, block, blocks);
        }

        public CustomPath merge(Cmds block, Cmds... blocks) {
            return new CustomPath(this::getTraj, false, 0.0, block, blocks);
        }

        public CustomPath asCustom() {
            return new CustomPath(this::getTraj);
        }
    }

    public static class CustomPath implements Block {
        private final Supplier<PathPlannerTrajectory> trajSupplier;
        private final Map<Double, List<Cmds>> cmdMap = new HashMap<>();
        private Boolean resetPose = false;

        public CustomPath(Supplier<PathPlannerTrajectory> trajSupplier) {
            this.trajSupplier = trajSupplier;
        }

        public CustomPath(Supplier<PathPlannerTrajectory> trajSupplier, boolean resetPose) {
            this.trajSupplier = trajSupplier;
            this.resetPose = resetPose;
        }

        public CustomPath(
                Supplier<PathPlannerTrajectory> trajSupplier,
                Boolean resetPose,
                Double percentThrough,
                Cmds block,
                Cmds... blocks) {
            this.trajSupplier = trajSupplier;
            this.resetPose = resetPose;
            var list = new ArrayList<Cmds>();
            list.add(block);
            list.addAll(Arrays.asList(blocks));
            cmdMap.put(percentThrough, list);
        }

        public CustomPath resetPose() {
            this.resetPose = true;
            return this;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {

            var traj = trajSupplier.get();

            var markers = new ArrayList<EventMarker>();
            for (var cmdEntry : cmdMap.entrySet()) {
                var percThrough = cmdEntry.getKey();
                var cmds = cmdEntry.getValue();
                var names = new ArrayList<String>();
                for (var cmd : cmds) {
                    names.add(ScreamingSnakeToCamal(cmd.name()));
                }
                System.out.println(percThrough + ": " + names);
                markers.add(EventMarker.fromTime(names, traj.getTotalTimeSeconds() * percThrough));
            }

            markers.sort(Comparator.comparingDouble(i -> i.timeSeconds));

            traj = new PathPlannerTrajectory(
                    traj.getStates(),
                    markers,
                    new StopEvent(),
                    new StopEvent(),
                    false);

            if (resetPose) {
                return builder.resetPose(traj)
                        .andThen(builder.followPathWithEvents(traj));
            } else {
                return builder.followPathWithEvents(traj);
            }
        }

        @Override
        public String getCommandName() {
            return "Custom Path";
        }

        public CustomPath merge(Double percentThrough, Cmds block, Cmds... blocks) {
            if (cmdMap.containsKey(percentThrough)) {
                cmdMap.get(percentThrough).add(block);
                cmdMap.get(percentThrough).addAll(Arrays.asList(blocks));
            } else {
                var list = new ArrayList<Cmds>();
                list.add(block);
                list.addAll(Arrays.asList(blocks));
                cmdMap.put(percentThrough, list);
            }
            return this;
        }

        public CustomPath merge(Cmds block, Cmds... blocks) {
            return this.merge(0.0, block, blocks);
        }
    }

    public static class AlliancePath implements Block {
        private final CustomPath trajSupplierRed, trajSupplierBlue;

        public AlliancePath(CustomPath blue, CustomPath red) {
            this.trajSupplierBlue = blue;
            this.trajSupplierRed = red;
        }

        public AlliancePath(CustomPath blue, PPPaths red) {
            this(blue, red.asCustom());
        }

        public AlliancePath(PPPaths blue, CustomPath red) {
            this(blue.asCustom(), red);
        }

        public AlliancePath(PPPaths blue, PPPaths red) {
            this(blue.asCustom(), red.asCustom());
        }

        public AlliancePath merge(Double percentThrough, Cmds block, Cmds... blocks) {
            trajSupplierBlue.merge(percentThrough, block, blocks);
            trajSupplierRed.merge(percentThrough, block, blocks);
            return this;
        }

        public AlliancePath merge(Cmds block, Cmds... blocks) {
            return this.merge(0.0, block, blocks);
        }

        public AlliancePath resetPose() {
            trajSupplierBlue.resetPose();
            trajSupplierRed.resetPose();
            return this;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            if (DriverStation.getAlliance() == Alliance.Blue) {
                return trajSupplierBlue.getCommand(builder);
            } else {
                return trajSupplierRed.getCommand(builder);
            }
        }

        @Override
        public String getCommandName() {
            return "Alliance Path";
        }
    }

    public static enum DynPaths implements Block {
        BALANCE_FORWARD(new InstantCommand()),
        BALANCE_RIGHTWARD(new InstantCommand()),
        BALANCE_LEFTWARD(new InstantCommand()),
        BALANCE_BACKWARD(new InstantCommand()),

        TAXI_OVER_BALANCE_FORWARD(new InstantCommand()),
        TAXI_OVER_BALANCE_RIGHTWARD(new InstantCommand()),
        TAXI_OVER_BALANCE_LEFTWARD(new InstantCommand()),
        TAXI_OVER_BALANCE_BACKWARD(new InstantCommand());

        public final Command command;

        DynPaths(Command command) {
            this.command = command;
        }

        @Override
        public Command getCommand(SwerveAutoBuilder builder) {
            return command;
        }

        @Override
        public String getCommandName() {
            return "DynPath: " + ScreamingSnakeToCamal(this.name());
        }
    }

    public static Block[] groupBlocks(Block... blocks) {
        return blocks;
    }

    public static Command buildBlocks(Block... blocks) {
        var builder = PathLoader.getPPAutoBuilder();
        List<Command> commands = new ArrayList<>();

        commands.add(
            RobotContainer.swerve.runOnce(() -> RobotContainer.swerve.setYaw(0.0)).withName("Reset Yaw")
            .alongWith(StateManager.setLastState(States.STANDBY))
        );

        var scheduler = CommandScheduler.getInstance();
        Integer count = 0;
        for (Block block : blocks) {
            var cmd = block.getCommand(builder);
            scheduler.removeComposedCommand(cmd);
            var namedCmd = cmd.withName("Block: " + count);
            commands.add(namedCmd);
            scheduler.removeComposedCommand(namedCmd);
            count++;
        }

        commands.add(SwerveCommands.commandStopDrives(RobotContainer.swerve).withName("Stop Swerve"));

        return Commands.sequence(commands.toArray(CommandBase[]::new));
    }
}
