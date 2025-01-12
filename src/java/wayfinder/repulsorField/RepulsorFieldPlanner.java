package wayfinder.repulsorField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {
  // static final List<Obstacle> FIELD_OBSTACLES =
  //     List.of(
  //         new TeardropObstacle(new Translation2d(5.56, 2.74), .8, 1.5, .25, .8, 2),
  //         new TeardropObstacle(new Translation2d(3.45, 4.07), .8, 1.5, .25, .8, 2),
  //         new TeardropObstacle(new Translation2d(5.56, 5.35), .8, 1.5, .25, .8, 2),
  //         new TeardropObstacle(new Translation2d(11.0, 2.74), .8, 1.5, .25, .8, 2),
  //         new TeardropObstacle(new Translation2d(13.27, 4.07), .8, 1.5, .25, .8, 2),
  //         new TeardropObstacle(new Translation2d(11.0, 5.35), .8, 1.5, .25, .8, 2));
  // static final List<Obstacle> WALLS =
  //     List.of(
  //         new HorizontalObstacle(0.0, 0.5, .5, true),
  //         new HorizontalObstacle(Constants.FIELD_WIDTH, 0.5, .5, false),
  //         new VerticalObstacle(0.0, 0.5, .5, true),
  //         new VerticalObstacle(Constants.FIELD_LENGTH, 0.5, .5, false));
  //      List.of();

  private final List<Obstacle> fixedObstacles = new ArrayList<>();

  public RepulsorFieldPlanner(Obstacle... obstacles) {
    fixedObstacles.addAll(List.of(obstacles));
  }

  Pose2d[] getArrows(
      Translation2d goal, double xCount, double yCount, double xLength, double yWidth) {
    Pose2d[] arrows = new Pose2d[(int) (xCount * yCount)];
    for (int x = 0; x <= xCount; x++) {
      for (int y = 0; y <= yCount; y++) {
        Translation2d translation = new Translation2d(x * xLength / xCount, y * yWidth / yCount);
        Translation2d force = getForce(translation, goal);
        if (force.getNorm() > 1e-6) {
          Rotation2d rotation = force.getAngle();

          arrows[x * (int) yCount + y] = new Pose2d(translation, rotation);
        }
      }
    }
    ;
    return arrows;
  }

  Translation2d getForce(Translation2d curLocation, Translation2d goal) {
    // push towards goal
    double xForceGoal = 0.0;
    double yForceGoal = 0.0;
    double xDisplacement = goal.getX() - curLocation.getX();
    double yDisplacement = goal.getY() - curLocation.getY();
    double norm = Math.hypot(xDisplacement, yDisplacement);
    if (norm != 0) {
      double cos = xDisplacement / norm;
      double sin = yDisplacement / norm;
      double mag = (1 + 1.0 / (1e-6 + norm));
      xForceGoal = mag * cos;
      yForceGoal = mag * sin;
    }

    // push away from obstacles
    double xForceObs = 0.0;
    double yForceObs = 0.0;
    for (Obstacle obs : fixedObstacles) {
      Translation2d force = obs.getForceAtPosition(curLocation, goal);
      xForceObs += force.getX();
      yForceObs += force.getY();
    }

    return new Translation2d(xForceGoal + xForceObs, yForceGoal + yForceObs);
  }

  // public Velocity2d sampleRepulsorField(Translation2d goal, Translation2d loc, double speed) {
  //   double stepSize_m = speed * 0.02;
  //   Translation2d err = loc.minus(goal);
  //   Translation2d netForce = getForce(loc, goal);
  //   // Calculate how quickly to move in this direction
  //   var closeToGoalMax = speed * Math.min(err.getNorm() / 2, 1);

  //   stepSize_m = Math.min(speed, closeToGoalMax) * 0.02;
  //   var step = new Translation2d(stepSize_m, netForce.getAngle());
  //   return sample(intermediateGoal, pose.getRotation(), step.getX() / 0.02, step.getY() / 0.02);
  // }

  public ArrayList<Translation2d> getTrajectory(
      Translation2d goal, Translation2d loc, double stepSize_m) {
    ArrayList<Translation2d> trajectory = new ArrayList<>();
    Translation2d robot = loc;
    for (int i = 0; i < 400; i++) {
      var err = robot.minus(goal);
      if (err.getNorm() < stepSize_m * 1.5) {
        trajectory.add(goal);
        break;
      } else {
        var netForce = getForce(robot, goal);
        if (netForce.getNorm() == 0) {
          break;
        }
        var step = new Translation2d(stepSize_m, netForce.getAngle());
        var intermediateGoal = robot.plus(step);
        trajectory.add(intermediateGoal);
        robot = intermediateGoal;
      }
    }
    return trajectory;
  }
}
