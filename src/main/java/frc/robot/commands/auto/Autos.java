package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.Blocks.Block;
import static frc.robot.commands.auto.Blocks.Cmds;
import static frc.robot.commands.auto.Blocks.PPPaths;

public class Autos {

    public static final Block[] THREE_GAME_PIECE_FLAT_CUBE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_MID,
        PPPaths.PLACE9_FLAT.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_SWOOP4B
            .merge(0.1, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE)
            .merge(0.65, Cmds.PLACE_STANDBY),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_HIGH_NO_DROP),
        Cmds.PLACE_HIGH,
        PPPaths.PLACE8_FLAT.merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.FLAT_PICKUP3.merge(
            0.2, Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
        PPPaths.PICKUP3_FLAT.merge(Cmds.STOW).merge(0.7, Cmds.PLACE_STANDBY),
        PPPaths.FLAT_PLACE8.merge(Cmds.PLACE_MID),
        Cmds.GYRO_180,
        Cmds.STOW
    );

    public static final Block[] PLACE_TAXI_WIRE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE1_WIRE.resetPose().merge(Cmds.STOW, Cmds.DESIRE_CUBE),
        PPPaths.WIRE_OVER_OUT,
        PPPaths.WIRE_PICKUP1.merge(Cmds.PICKUP_GROUND, Cmds.OVERRIDE_HOLD_CUBE),
        Cmds.GYRO_0,
        Cmds.STOW
    );

    public static final Block[] PLACE_BALANCE = Blocks.groupBlocks(
        Cmds.OVERRIDE_HOLD_CONE,
        Cmds.PLACE_HIGH,
        PPPaths.PLACE_BAL.resetPose().merge(Cmds.STOW),
        Cmds.X_WHEELS
    );

    public enum AutoRoutines {
        NOTHING(new Block[] {}),
        THREE_GAME_PIECE_FLAT(Autos.THREE_GAME_PIECE_FLAT_CUBE),
        PLACE_TAXI_WIRE(Autos.PLACE_TAXI_WIRE),
        PLACE_BALANCE(Autos.PLACE_BALANCE);

        private final Block[] blocks;

        private AutoRoutines(Block... blocks) {
            this.blocks = blocks;
        }

        public Command getCommand() {
            return Blocks.buildBlocks(blocks).withName(this + "(" + DriverStation.getAlliance() + ") Auto");
        }
    }
}
