package igknighters.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.Localizer;
import igknighters.Robot;
import igknighters.SimCtx;
import igknighters.constants.ConstValues.kSwerve;
import igknighters.constants.ConstValues.kVision;
import igknighters.constants.FieldConstants;
import igknighters.constants.RobotConfig;
import igknighters.subsystems.Subsystems.SharedSubsystem;
import igknighters.subsystems.vision.camera.Camera;
import igknighters.subsystems.vision.camera.Camera.CameraConfig;
import igknighters.subsystems.vision.camera.CameraRealPhoton;
import igknighters.subsystems.vision.camera.CameraSimPhoton;
import igknighters.util.plumbing.Channel.Sender;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import monologue.Annotations.IgnoreLogged;
import monologue.ProceduralStructGenerator;
import wpilibExt.Tracer;

public class Vision implements SharedSubsystem {
  @IgnoreLogged private final Localizer localizer;

  private final Sender<VisionSample> visionSender;

  private final Camera[] cameras;

  private final HashSet<Integer> seenTags = new HashSet<>();

  public record VisionUpdateFaults(
      boolean extremeJitter,
      boolean outOfBounds,
      boolean outOfRange,
      boolean infeasibleZValue,
      boolean infeasiblePitchValue,
      boolean infeasibleRollValue,
      boolean sketchyTags,
      boolean noTags,
      boolean singleTag)
      implements StructSerializable {

    private static final VisionUpdateFaults kEmpty =
        new VisionUpdateFaults(false, false, false, false, false, false, false, false, false);

    public static VisionUpdateFaults empty() {
      return kEmpty;
    }

    public static VisionUpdateFaults solve(
        Pose3d pose,
        Pose3d lastPose,
        double time,
        double avgDistance,
        List<Integer> tagsList,
        List<Integer> sketchyTagsList) {
      Translation2d simplePose = pose.getTranslation().toTranslation2d();
      boolean outOfBounds =
          simplePose.getX() < 0.0
              || simplePose.getX() > FieldConstants.FIELD_LENGTH
              || simplePose.getY() < 0.0
              || simplePose.getY() > FieldConstants.FIELD_WIDTH
              || Double.isNaN(simplePose.getX())
              || Double.isNaN(simplePose.getY());
      boolean extremeJitter =
          pose.getTranslation().getDistance(lastPose.getTranslation())
              > time * kSwerve.MAX_DRIVE_VELOCITY;
      boolean infeasibleZValue = Math.abs(pose.getTranslation().getZ()) > kVision.MAX_Z_DELTA;
      boolean infeasiblePitchValue = pose.getRotation().getY() > kVision.MAX_ANGLE_DELTA;
      boolean infeasibleRollValue = pose.getRotation().getX() > kVision.MAX_ANGLE_DELTA;
      boolean outOfRange = avgDistance > 5.5;
      boolean noTags = tagsList.size() == 0;
      boolean singleTag = tagsList.size() == 1;
      boolean sketchyTags = tagsList.stream().anyMatch(sketchyTagsList::contains);
      return new VisionUpdateFaults(
          extremeJitter,
          outOfBounds,
          outOfRange,
          infeasibleZValue,
          infeasiblePitchValue,
          infeasibleRollValue,
          sketchyTags,
          noTags,
          singleTag);
    }

    public static final Struct<VisionUpdateFaults> struct =
        ProceduralStructGenerator.genRecord(VisionUpdateFaults.class);
  }

  public record VisionUpdate(Pose3d pose, double timestamp, VisionUpdateFaults faults)
      implements StructSerializable {

    private static final VisionUpdate kEmpty =
        new VisionUpdate(Pose3d.kZero, 0.0, VisionUpdateFaults.empty());

    public static VisionUpdate empty() {
      return kEmpty;
    }

    public static final Struct<VisionUpdate> struct =
        ProceduralStructGenerator.genRecord(VisionUpdate.class);
  }

  public record VisionSample(Pose2d pose, double timestamp, double trust)
      implements StructSerializable {

    public static final Struct<VisionSample> struct =
        ProceduralStructGenerator.genRecord(VisionSample.class);
  }

  private Camera makeCamera(CameraConfig config, SimCtx simCtx) {
    if (Robot.isSimulation()) {
      return new CameraSimPhoton(config.cameraName(), config.cameraPose(), simCtx);
    } else {
      return new CameraRealPhoton(config.cameraName(), config.cameraPose());
    }
  }

  public Vision(final Localizer localizer, final SimCtx simCtx) {
    this.localizer = localizer;
    final var configs = kVision.CameraConfigs.forRobot(RobotConfig.getRobotID());
    this.cameras = new Camera[configs.length];
    for (int i = 0; i < configs.length; i++) {
      this.cameras[i] = makeCamera(configs[i], simCtx);
    }

    visionSender = localizer.visionDataSender();
  }

  private Optional<VisionSample> gaugeTrust(final VisionUpdate update) {
    final VisionUpdateFaults faults = update.faults();

    // These are "fatal" faults that should always invalidate the update
    if (faults.noTags
        || (faults.sketchyTags && faults.singleTag)
        || faults.outOfBounds
        || faults.extremeJitter) {
      return Optional.empty();
    }

    double trust = kVision.ROOT_TRUST;

    // If the average distance of the tags is too far away reduce the trust
    if (faults.outOfRange()) {
      trust /= 2.0;
    }

    // If any tags seen are sketchy reduce the trust
    // If the only tag seen is sketchy massively reduce the trust
    if (faults.singleTag || faults.sketchyTags) {
      trust /= 3.0;
    } else if (faults.sketchyTags) {
      trust /= 1.5;
    }

    // Completely arbitrary values for the velocity thresholds.
    // When the robot is moving fast there can be paralaxing and motion blur
    // that can cause the vision system to be less accurate, reduce the trust due to this
    ChassisSpeeds velo = localizer.speeds();
    if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm()
        > kSwerve.MAX_DRIVE_VELOCITY / 2.0) {
      trust /= 2.0;
    }
    if (velo.omegaRadiansPerSecond > kSwerve.MAX_ANGULAR_VELOCITY / 3.0) {
      trust /= 2.0;
    }

    // If the vision rotation varies significantly from the gyro rotation reduce the trust
    Rotation2d rotation = localizer.pose().getRotation();
    if (Math.abs(
            MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(update.pose().getRotation().toRotation2d().getRadians()))
        > Math.toRadians(5.0)) {
      trust /= 2.0;
    }

    if (faults.infeasibleZValue || faults.infeasiblePitchValue || faults.infeasibleRollValue) {
      trust /= 2.0;
    }

    return Optional.of(new VisionSample(update.pose().toPose2d(), update.timestamp(), trust));
  }

  @Override
  public void periodic() {
    Tracer.startTrace("VisionPeriodic");

    for (final Camera camera : cameras) {
      Tracer.startTrace(camera.getName() + "Periodic");
      camera.periodic();

      camera.flushUpdates().stream()
          .map(this::gaugeTrust)
          .filter(Optional::isPresent)
          .map(Optional::get)
          .forEach(visionSender::send);

      seenTags.addAll(camera.getSeenTags());

      Tracer.endTrace();
    }

    log(
        "seenTags",
        seenTags.stream()
            .map(i -> FieldConstants.APRIL_TAG_FIELD.getTagPose(i))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toArray(Pose3d[]::new));
    seenTags.clear();

    Tracer.endTrace();
  }
}
