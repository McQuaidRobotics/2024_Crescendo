package frc.robot.commands.auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.kAuto;

public class PathLoader {

    public static PathPlannerTrajectory openFilePath(String autoPathFile, Double maxSpeed) {
        //if filename ends with _BI do nothing
        if(!autoPathFile.endsWith("_BI") && DriverStation.getAlliance() == Alliance.Red) {
            autoPathFile += "_R";
        }
        PathPlannerTrajectory autoPath = PathPlanner.loadPath(
            autoPathFile, 
            new PathConstraints(maxSpeed, 3.0)
        );
        return autoPath;
    }

    public static SwerveAutoBuilder getPPAutoBuilder() {
        return new SwerveAutoBuilder(
            RobotContainer.swerve::getPose, 
            RobotContainer.swerve::resetOdometry,
            kAuto.AUTO_TRANSLATION_PID,
            kAuto.AUTO_ANGULAR_PID,
            RobotContainer.swerve::driveRobotRelative,
            Blocks.EVENT_MAP,
            false,
            RobotContainer.swerve);
    }
}
