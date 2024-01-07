package com.igknighters.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.igknighters.Robot;
import com.igknighters.Constants.kSwerve;
import com.igknighters.constants.ConstValues;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Field2d field = new Field2d();

    private final Pigeon2 gyro;
    private final Pigeon2SimState gyroSim;
    private final StatusSignal<Double> gyroRollSignal;
    private final StatusSignal<Double> gyroPitchSignal;
    private final StatusSignal<Double> gyroYawSignal;

    public Swerve() {

        gyro = new Pigeon2(ConstValues.kSwerve.PIGEON_ID, ConstValues.kSwerve.CANBUS);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyroSim = gyro.getSimState();
        setYaw(0.0);
        gyroRollSignal = gyro.getRoll();
        gyroPitchSignal = gyro.getPitch();
        gyroYawSignal = gyro.getYaw();

        swerveMods = Robot.isReal() ? new SwerveModule[] {
                new SwerveModuleReal(ConstValues.kSwerve.Mod0.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod1.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod2.CONSTANTS),
                new SwerveModuleReal(ConstValues.kSwerve.Mod3.CONSTANTS)
        }
        : new SwerveModule[] {
                new SwerveModuleSim(ConstValues.kSwerve.Mod0.CONSTANTS),
                new SwerveModuleSim(ConstValues.kSwerve.Mod1.CONSTANTS),
                new SwerveModuleSim(ConstValues.kSwerve.Mod2.CONSTANTS),
                new SwerveModuleSim(ConstValues.kSwerve.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDriveOdometry(
                kSwerve.SWERVE_KINEMATICS,
                getYawRot(),
                getModulePositions());

        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] mSwerveModuleStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYawRot())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.getModuleNumber()], isOpenLoop);
        }
    }

    public void drive(Translation2d translation, Translation2d absRotation, boolean isOpenLoop) {
        // the angle of the translation vector
        Double wantedAngle = Math.atan2(absRotation.getY(), absRotation.getX());
        // a 0-1 value representing the magnitude of the translation vector
        Double magnitude = absRotation.getNorm();
        // the current angle reading of the gyro
        Double currentAngle = getYawRot().getRadians();
        // the angle of the translation vector relative to the gyro
        Double relativeAngle = wantedAngle - currentAngle;

        Double rotVelo;
        if (relativeAngle < kSwerve.MAX_ANGULAR_VELOCITY * 0.02) {
            rotVelo = relativeAngle * 50;
        } else {
            rotVelo = Math.signum(relativeAngle) * kSwerve.MAX_ANGULAR_VELOCITY * magnitude;
        }

        SwerveModuleState[] mSwerveModuleStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX() * kSwerve.MAX_SPEED,
                        translation.getY() * kSwerve.MAX_SPEED,
                        rotVelo,
                        getYawRot()));

        SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(mSwerveModuleStates[module.getModuleNumber()], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        if (Robot.isReal())
            speeds.omegaRadiansPerSecond *= -1;
        SwerveModuleState[] targetStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kSwerve.MAX_SPEED);

        SmartDashboard.putNumber("Speed X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Speed Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Speed Rot", speeds.omegaRadiansPerSecond);

        setModuleStates(targetStates);
    }

    public void setYaw(double val) {
        gyro.setYaw(val);
    }

    public Rotation2d getYawRot() {
        return Rotation2d.fromDegrees(scope0To360(this.getYaw()));
    }

    public Double getYaw() {
        return gyro.getYaw().getValue();
    }

    public Double getPitch() {
        return gyro.getPitch().getValue();
    }

    public Double getRoll() {
        return gyro.getRoll().getValue();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (SwerveModule module : swerveMods) {
            modulePositions[module.getModuleNumber()] = module.getCurrentPosition();
        }
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ConstValues.kSwerve.MAX_DRIVE_VELOCITY);

        for (SwerveModule module : swerveMods) {
            module.setDesiredState(desiredStates[module.getModuleNumber()], false);
        }
    }

    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds));
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : swerveMods) {
            states[module.getModuleNumber()] = module.getCurrentState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYawRot(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        gyroPitchSignal.refresh();
        gyroRollSignal.refresh();
        gyroYawSignal.refresh();
        for (SwerveModule module : swerveMods) {
            module.periodic();
        }

        var currCmd = this.getCurrentCommand();
        SmartDashboard.putString("swerve cmd", currCmd == null ? "None" : currCmd.getName());

        var modulePoses = getModulePositions();
        for (var i = 0; i < modulePoses.length; i++) {
            SmartDashboard.putNumber("Module " + i + " Distance", modulePoses[i].distanceMeters);
            SmartDashboard.putNumber("Module " + i + " Angle", modulePoses[i].angle.getDegrees());
        }

        var gyroRot = getYawRot();
        SmartDashboard.putNumber("Gyro Angle", gyroRot.getDegrees());

        var pose = swerveOdometry.update(gyroRot, modulePoses);
        field.getRobotObject().setPose(pose);
    }

    @Override
    public void simulationPeriodic() {

        ChassisSpeeds currentSpeeds = kSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        gyroSim.setRawYaw(
                getYawRot().getDegrees() + (Units.radiansToDegrees(currentSpeeds.omegaRadiansPerSecond) * 0.02));
    }

    public static double scope0To360(double angle) {
        if (angle < 0) {
            angle = 360 - (Math.abs(angle) % 360);
        } else {
            angle %= 360;
        }
        return angle;
    }
}
