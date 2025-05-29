package com.igknighters.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.TreeMap;

public class TwistyPoseEst {
  public record VisionScalars(
      double thresholdForTheta, double thetaScalar, double translationScalar) {
    public VisionScalars {
      thresholdForTheta = MathUtil.clamp(thresholdForTheta, 0.0, 1.0);
      thetaScalar = MathUtil.clamp(thetaScalar, 0.0, 1.0);
      translationScalar = MathUtil.clamp(translationScalar, 0.0, 1.0);
    }
  }

  private final class TimestampedTwist2d extends Twist2d {
    public final double timestamp;

    public TimestampedTwist2d(double dx, double dy, double dtheta, double timestamp) {
      super(dx, dy, dtheta);
      this.timestamp = timestamp;
    }

    @Override
    public boolean equals(Object obj) {
      if (obj instanceof TimestampedTwist2d other) {
        return super.equals(obj) && MathUtil.isNear(other.timestamp, timestamp, 1E-9);
      }
      return false;
    }
  }

  /**
   * This is a class to optimize allocation of geometry objects.
   *
   * <p>It mirrors the data contained in a {@link Pose2d} object but with interior mutability and
   * only primitive fields.
   */
  private final class PrimitivePose {
    public double x, y, sin, cos;

    public PrimitivePose(Pose2d pose) {
      x = pose.getTranslation().getX();
      y = pose.getTranslation().getY();
      sin = pose.getRotation().getSin();
      cos = pose.getRotation().getCos();
    }

    public Pose2d toPose2d() {
      return new Pose2d(new Translation2d(x, y), new Rotation2d(cos, sin));
    }

    /**
     * An implementation of {@link Pose2d#exp(Twist2d)} that mutates the object.
     *
     * @param twist the twist to apply
     */
    public void twistBy(Twist2d twist) {
      double dx = twist.dx;
      double dy = twist.dy;
      double dtheta = twist.dtheta;

      double transformSin = Math.sin(dtheta);
      double transformCos = Math.cos(dtheta);

      double s;
      double c;
      if (Math.abs(dtheta) < 1E-9) {
        s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
        c = 0.5 * dtheta;
      } else {
        s = transformSin / dtheta;
        c = (1 - transformCos) / dtheta;
      }

      double transformX = (dx * s) - (dy * c);
      double transformY = (dx * c) + (dy * s);

      double rotatedX = (transformX * cos) - (transformY * sin);
      double rotatedY = (transformX * sin) + (transformY * cos);

      x += rotatedX;
      y += rotatedY;

      double newCos = (transformCos * cos) - (transformSin * sin);
      double newSin = (transformCos * sin) + (transformSin * cos);

      cos = newCos;
      sin = newSin;
    }
  }

  private final TreeMap<Double, TimestampedTwist2d> samples = new TreeMap<>();
  private Pose2d rootPose = Pose2d.kZero;
  private Object prevWheelPositions;
  private Rotation2d prevGyroAngle;

  public void resetPose(Pose2d pose) {
    rootPose = pose;
    samples.clear();
    prevGyroAngle = null;
    samples.put(
        Timer.getFPGATimestamp(), new TimestampedTwist2d(0.0, 0.0, 0.0, Timer.getFPGATimestamp()));
  }

  private PrimitivePose poseAtTimestampPrimitive(double timestamp) {
    PrimitivePose pose = new PrimitivePose(rootPose);
    for (TimestampedTwist2d sample : samples.values()) {
      if (sample.timestamp > timestamp) {
        break;
      }
      pose.twistBy(sample);
    }
    return pose;
  }

  private Pose2d poseAtTimestamp(double timestamp) {
    return poseAtTimestampPrimitive(timestamp).toPose2d();
  }

  public void prune(double maxAge) {
    final double pruneBefore = Timer.getFPGATimestamp() - maxAge;
    rootPose = poseAtTimestamp(pruneBefore);
    samples.values().removeIf(sample -> sample.timestamp < pruneBefore);
  }

  private void forcePrune(double pruneBefore, Pose2d newRoot) {
    rootPose = newRoot;
    samples.values().removeIf(sample -> sample.timestamp < pruneBefore);
  }

  private double oldestTimestamp() {
    if (samples.isEmpty()) {
      return 0.0;
    } else {
      return samples.firstKey();
    }
  }

  /**
   * Adds a sample to the estimator
   *
   * @param pose the pose of the robot at the time of the sample
   * @param timestamp the timestamp of the sample
   * @param weight the weight of the sample (0.0 to 1.0)
   */
  public void addVisionSample(VisionScalars scalars, Pose2d pose, double timestamp, double weight) {
    weight = MathUtil.clamp(weight, 0.0, 1.0);
    if (timestamp < oldestTimestamp()) {
      return;
    }
    Pose2d lastPose = poseAtTimestamp(timestamp);
    Twist2d twist = lastPose.log(pose);
    twist.dx *= weight * scalars.translationScalar;
    twist.dy *= weight * scalars.translationScalar;
    if (weight > scalars.thresholdForTheta) {
      twist.dtheta *= scalars.thetaScalar * weight;
    } else {
      twist.dtheta *= 0.0;
    }
    forcePrune(timestamp, lastPose.exp(twist));
  }

  @SuppressWarnings("unchecked")
  public <T> void addDriveSample(
      Kinematics<?, T> kinematics,
      T wheelPositions,
      Rotation2d gyroAngle,
      double timestamp,
      double weight) {
    if (timestamp < oldestTimestamp()) {
      return;
    }
    weight = MathUtil.clamp(weight, 0.0, 1.0);
    if (prevWheelPositions == null || prevGyroAngle == null) {
      prevWheelPositions = wheelPositions;
      prevGyroAngle = gyroAngle;
      return;
    }
    Twist2d twist = kinematics.toTwist2d((T) prevWheelPositions, wheelPositions);
    samples.put(
        timestamp,
        new TimestampedTwist2d(
            twist.dx * weight,
            twist.dy * weight,
            gyroAngle.minus(prevGyroAngle).getRadians(),
            timestamp));
    prevWheelPositions = wheelPositions;
    prevGyroAngle = gyroAngle;
  }

  public Pose2d getEstimatedPose() {
    return poseAtTimestamp(Timer.getFPGATimestamp());
  }

  public Optional<Pose2d> getEstimatedPoseFromPast(double secondsAgo) {
    double timestamp = Timer.getFPGATimestamp() - secondsAgo;
    if (timestamp < oldestTimestamp()) {
      return Optional.empty();
    }
    return Optional.of(poseAtTimestamp(Timer.getFPGATimestamp() - secondsAgo));
  }
}
