package wayfinder.repulsorField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wpilibExt.MutTranslation2d;

public abstract class Obstacle {
  double strength;
  boolean positive;

  public Obstacle(double strength, boolean positive) {
    this.strength = strength;
    this.positive = positive;
  }

  public abstract MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal);

  protected double distToForceMag(double dist, double maxRange) {
    if (MathUtil.isNear(0, dist, 1e-2)) {
      dist = 1e-2;
    }
    var forceMag = strength / (dist * dist);
    forceMag -= strength / (maxRange * maxRange);
    forceMag *= positive ? 1 : -1;
    return forceMag;
  }

  protected double rotateBy(double radians, double rhs) {
    return Math.cos(radians) * rhs;
  }

  static class PointObstacle extends Obstacle {
    final Translation2d loc;
    final double effectMaxRange = 0.5;

    final MutTranslation2d positionToLoc = new MutTranslation2d();
    final MutTranslation2d goalToPosition = new MutTranslation2d();
    final MutTranslation2d outwardsVector = new MutTranslation2d();
    final MutTranslation2d sidewaysVector = new MutTranslation2d();
    final MutTranslation2d output = new MutTranslation2d();

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      this.positionToLoc.set(position);
      this.positionToLoc.minusMut(loc);
      this.goalToPosition.set(goal);
      this.goalToPosition.minusMut(position);
      output.set(Translation2d.kZero);

      double dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return output;
      }
      double outwardsMag = distToForceMag(loc.getDistance(position), effectMaxRange);
      outwardsVector.setPolar(outwardsMag, positionToLoc.getRadians());
      // theta = angle between position->target vector and obstacle->position vector
      Rotation2d theta = goalToPosition.getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      sidewaysVector.set(outwardsVector);
      sidewaysVector.rotateByMut(Rotation2d.kCCW_90deg);
      sidewaysVector.divMut(outwardsVector.getNorm());
      sidewaysVector.timesMut(mag);

      output.set(outwardsVector);
      output.plusMut(sidewaysVector);
      return output;
    }
  }

  static class SnowmanObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double secondaryDistance;
    final double secondaryMaxRange;
    final double secondaryStrengthRatio;

    final MutTranslation2d goalToLoc = new MutTranslation2d();
    final MutTranslation2d sidewaysCircle = new MutTranslation2d();
    final MutTranslation2d output = new MutTranslation2d();

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

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      output.set(Translation2d.kZero);
      goalToLoc.set(loc);
      goalToLoc.minusMut(goal);
      sidewaysCircle.setPolar(secondaryDistance, goalToLoc.getRadians());
      sidewaysCircle.plusMut(loc);
      double dist = loc.getDistance(position);
      double sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return output;
      }
      double sidewaysMag =
          distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange)
              / secondaryStrengthRatio;
      double outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
      var initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          goal.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      goalToLoc.rotateByMut(Rotation2d.kCCW_90deg);
      double sidewaysAngle = goalToLoc.getRadians();
      output.setPolar(sideways, sidewaysAngle);
      output.plusMut(initial);
      return output;
    }
  }

  static class TeardropObstacle extends Obstacle {
    final Translation2d loc;
    final double primaryMaxRange;
    final double primaryRadius;
    final double tailStrength;
    final double tailDistance;

    final MutTranslation2d output = new MutTranslation2d();

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

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      var targetToLoc = loc.minus(goal);
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
              goal.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
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

      output.set(outwardsForce);
      output.plusMut(sidewaysForce);
      return output;
    }
  }

  static class HorizontalObstacle extends Obstacle {
    final double y;
    final double maxRange;

    final MutTranslation2d output = new MutTranslation2d();

    public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.y = y;
      this.maxRange = maxRange;
    }

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      output.set(Translation2d.kZero);
      var dist = Math.abs(position.getY() - y);
      if (dist < maxRange) {
        output.set(0, distToForceMag(y - position.getY(), maxRange));
      }
      return output;
    }
  }

  static class VerticalObstacle extends Obstacle {
    final double x;
    final double maxRange;

    final MutTranslation2d output = new MutTranslation2d();

    public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
      super(strength, positive);
      this.x = x;
      this.maxRange = maxRange;
    }

    public MutTranslation2d getForceAtPosition(Translation2d position, Translation2d goal) {
      output.set(Translation2d.kZero);
      var dist = Math.abs(position.getX() - x);
      if (dist < maxRange) {
        output.set(distToForceMag(x - position.getX(), maxRange), 0);
      }
      return output;
    }
  }
}
