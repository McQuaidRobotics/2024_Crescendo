package wayfinder.repulsorField;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength;
    boolean positive;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist, double maxRange) {
      if (MathUtil.isNear(0, dist, 1e-2)) {
        dist = 1e-2;
      }
      var forceMag = strength / (dist * dist);
      forceMag -= strength / (maxRange * maxRange);
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double effectMaxRange = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return Translation2d.kZero;
      }
      var outwardsMag = distToForceMag(loc.getDistance(position), effectMaxRange);
      var outwardsVector = new Translation2d(outwardsMag, position.minus(loc).getAngle());
      // theta = angle between position->target vector and obstacle->position vector
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      var sidewaysVector =
          outwardsVector.rotateBy(Rotation2d.kCCW_90deg).div(outwardsVector.getNorm()).times(mag);

      return outwardsVector.plus(sidewaysVector);
    }
  }

  static class SnowmanObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double secondaryDistance;
    final double secondaryMaxRange;
    final double secondaryStrengthRatio;

    public SnowmanObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double secondaryDistance,
        double secondaryStrength,
        double secondaryMaxRange) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.secondaryDistance = secondaryDistance;
      this.secondaryMaxRange = secondaryMaxRange;
      secondaryStrengthRatio = primaryStrength / secondaryStrength;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysCircle = new Translation2d(secondaryDistance, targetToLoc.getAngle()).plus(loc);
      var dist = loc.getDistance(position);
      var sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return Translation2d.kZero;
      }
      var sidewaysMag =
          distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange)
              / secondaryStrengthRatio;
      var outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
      var initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Translation2d(sideways, sidewaysAngle).plus(initial);
    }
  }

  static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailDistance;

    public TeardropObstacle(
        Translation2d loc,
        double primaryStrength,
        double primaryMaxRange,
        double primaryRadius,
        double tailStrength,
        double tailLength) {
      super(primaryStrength, true);
      this.loc = loc;
      this.primaryMaxRange = primaryMaxRange;
      this.primaryRadius = primaryRadius;
      this.tailStrength = tailStrength;
      this.tailDistance = tailLength + primaryMaxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      var sidewaysPoint = new Translation2d(tailDistance, targetToLoc.getAngle()).plus(loc);

      var positionToLocation = position.minus(loc);
      var positionToLocationDistance = positionToLocation.getNorm();
      Translation2d outwardsForce;
      if (positionToLocationDistance <= primaryMaxRange) {
        outwardsForce =
            new Translation2d(
                distToForceMag(
                    Math.max(positionToLocationDistance - primaryRadius, 0),
                    primaryMaxRange - primaryRadius),
                positionToLocation.getAngle());
      } else {
        outwardsForce = Translation2d.kZero;
      }

      var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
      var distanceAlongLine = positionToLine.getX();

      Translation2d sidewaysForce;
      var distanceScalar = distanceAlongLine / tailDistance;
      if (distanceScalar >= 0 && distanceScalar <= 1) {
        var secondaryMaxRange =
            MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        var distanceToLine = Math.abs(positionToLine.getY());
        if (distanceToLine <= secondaryMaxRange) {
          var sidewaysMag =
              tailStrength
                  * (1 - distanceScalar * distanceScalar)
                  * (secondaryMaxRange - distanceToLine);
          // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
          var sidewaysTheta =
              target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
          sidewaysForce =
              new Translation2d(
                  sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                  targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
        } else {
          sidewaysForce = Translation2d.kZero;
        }
      } else {
        sidewaysForce = Translation2d.kZero;
      }

      return outwardsForce.plus(sidewaysForce);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    final double y;
    final double maxRange;

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
    }
  }

  static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > maxRange) {
        return Translation2d.kZero;
      }
      return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
    }
  }

  static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new TeardropObstacle(new Translation2d(5.56, 2.74), .8, 1.5, .25, .8, 2),
          new TeardropObstacle(new Translation2d(3.45, 4.07), .8, 1.5, .25, .8, 2),
          new TeardropObstacle(new Translation2d(5.56, 5.35), .8, 1.5, .25, .8, 2),
          new TeardropObstacle(new Translation2d(11.0, 2.74), .8, 1.5, .25, .8, 2),
          new TeardropObstacle(new Translation2d(13.27, 4.07), .8, 1.5, .25, .8, 2),
          new TeardropObstacle(new Translation2d(11.0, 5.35), .8, 1.5, .25, .8, 2));
  static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 0.5, .5, true),
          new HorizontalObstacle(Constants.FIELD_WIDTH, 0.5, .5, false),
          new VerticalObstacle(0.0, 0.5, .5, true),
          new VerticalObstacle(Constants.FIELD_LENGTH, 0.5, .5, false));
  //      List.of();

  private final List<Obstacle> fixedObstacles = new ArrayList<>();
  private Translation2d goal = new Translation2d(1, 1);

  private static final int ARROWS_X = 40;
  private static final int ARROWS_Y = 20;

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
  }

  private final Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  private Translation2d lastGoal;
  private Pose2d[] arrowList;

  // A grid of arrows drawn in AScope
  Pose2d[] getArrows() {
    if (goal.equals(lastGoal)) {
      return arrowList;
    }
    var list = new ArrayList<Pose2d>();
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(
                x * Constants.FIELD_LENGTH / ARROWS_X, y * Constants.FIELD_WIDTH / ARROWS_Y);
        var force = getForce(translation, goal);
        if (force.getNorm() > 1e-6) {
          var rotation = force.getAngle();

          list.add(new Pose2d(translation, rotation));
        }
      }
    }
    lastGoal = goal;
    arrowList = list.toArray(new Pose2d[0]);
    return arrowList;
  }

  Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Translation2d();
    }
    var direction = displacement.getAngle();
    var mag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
    return new Translation2d(mag, direction);
  }

  Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Translation2d.kZero;
    for (Obstacle obs : fixedObstacles) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Translation2d getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
  }

  private SwerveSample sample(Translation2d trans, Rotation2d rot, double vx, double vy) {
    return new SwerveSample(
        0,
        trans.getX(),
        trans.getY(),
        rot.getRadians(),
        vx,
        vy,
        0,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
  }

  public SwerveSample sampleRepulsorField(Pose2d pose, double maxSpeed) {
    double stepSize_m = maxSpeed * Robot.defaultPeriodSecs;
    var curTrans = pose.getTranslation();
    var err = curTrans.minus(goal);
    if (err.getNorm() < stepSize_m * 1.5) {
      return sample(goal, pose.getRotation(), 0, 0);
    } else {
      var netForce = getForce(curTrans, goal);
      // Calculate how quickly to move in this direction
      var closeToGoalMax = maxSpeed * Math.min(err.getNorm() / 2, 1);

      stepSize_m = Math.min(maxSpeed, closeToGoalMax) * Robot.defaultPeriodSecs;
      var step = new Translation2d(stepSize_m, netForce.getAngle());
      var intermediateGoal = curTrans.plus(step);
      return sample(intermediateGoal, pose.getRotation(), step.getX() / 0.02, step.getY() / 0.02);
    }
  }

  public ArrayList<Translation2d> getTrajectory(Translation2d current, double stepSize_m) {
    ArrayList<Translation2d> trajectory = new ArrayList<>();
    Translation2d robot = current;
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