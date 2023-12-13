package frc.robot.util;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.SwerveModuleConstants.ModuleId;
import frc.robot.Robot;

public class NTpreferences {
    private static HashMap<String, Double> universalOffsets = new HashMap<String, Double>();
    private static boolean loadedCorrectly = true;

    public static void loadPreferences() {
        loadedCorrectly = true;

        if (Robot.isReal()) {
            universalOffsets.put("0U", Preferences.getDouble("Module 0 Offset Universal", 0));
            universalOffsets.put("1U", Preferences.getDouble("Module 1 Offset Universal", 0));
            universalOffsets.put("2U", Preferences.getDouble("Module 2 Offset Universal", 0));
            universalOffsets.put("3U", Preferences.getDouble("Module 3 Offset Universal", 0));

            for (Double angle : universalOffsets.values()) {
                if ((angle <= 0 || angle >= 360) && Robot.isReal())
                    loadedCorrectly = false;
            }
        } else {
            universalOffsets.put("0U", 0.0);
            universalOffsets.put("1U", 0.0);
            universalOffsets.put("2U", 0.0);
            universalOffsets.put("3U", 0.0);
        }
    }

    public static boolean isOk() {
        return loadedCorrectly;
    }

    public static Rotation2d getRotationOffset(ModuleId module) {
        if (module == ModuleId.m3)
            return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("3U") + 225));
        else if (module == ModuleId.m0)
            return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("0U") + 315));
        else if (module == ModuleId.m2)
            return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("2U") + 125));
        else if (module == ModuleId.m1)
            return Rotation2d.fromDegrees(findCoterminalAngle(universalOffsets.get("1U") + 45));
        throw new IllegalArgumentException("Module not found");
    }

    public static double findCoterminalAngle(double angleOffset) {
        return (angleOffset > 360) ? angleOffset % 360 : angleOffset;
    }
}
