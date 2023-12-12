package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {

    public enum AutoRoutines {
        NOTHING(""),
        THREE_GAME_PIECE_FLAT(""),
        PLACE_TAXI_WIRE(""),
        PLACE_BALANCE("");

        final String name;

        private AutoRoutines(String name) {
            this.name = name;
        }

        public Command getCommand() {
            return Commands.none();
        }
    }
}
