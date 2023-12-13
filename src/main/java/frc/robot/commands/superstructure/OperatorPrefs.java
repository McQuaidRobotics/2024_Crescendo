package frc.robot.commands.superstructure;

public class OperatorPrefs {

    public static enum PickupMode {
        GROUND,
        STATION;

        private static PickupMode currentMode = GROUND;

        public static void setCurrentMode(PickupMode mode) {
            currentMode = mode;
        }

        public static PickupMode getCurrentMode() {
            return currentMode;
        }
    }

    public static enum ScoreLevel {
        LOW_FRONT,
        LOW_BACK,
        MID,
        HIGH;

        private static ScoreLevel currentLevel = HIGH;

        public static void setCurrentLevel(ScoreLevel level) {
            currentLevel = level;
        }

        public static ScoreLevel getCurrentLevel() {
            return currentLevel;
        }
    }

    public static Boolean NEED_HOME = false;

}
