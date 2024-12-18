package com.igknighters.subsystems.swerve;

// import java.util.function.IntFunction;

import com.igknighters.constants.ConstValues.kSwerve;
import com.igknighters.util.geom.GeomUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import monologue.Logged;
import monologue.ProceduralStructGenerator;
import monologue.Annotations.Log;

public class SlipDetection implements Logged {
    private static record ModuleData (
        double mps,
        double distance,
        Rotation2d angleRads,
        Translation2d position
    ) implements StructSerializable {
        public static ModuleData zeroed() {
            return new ModuleData(0, 0, GeomUtil.ROTATION2D_ZERO, GeomUtil.TRANSLATION2D_ZERO);
        }

        @SuppressWarnings("unused")
        public static final Struct<ModuleData> struct = ProceduralStructGenerator.genRecord(ModuleData.class);
    }

    @Log
    private final ModuleData[] modules = new ModuleData[] {
        ModuleData.zeroed(),
        ModuleData.zeroed(),
        ModuleData.zeroed(),
        ModuleData.zeroed()
    };
    @Log
    private final SwerveModulePosition[] lastPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(0, GeomUtil.ROTATION2D_ZERO),
        new SwerveModulePosition(0, GeomUtil.ROTATION2D_ZERO),
        new SwerveModulePosition(0, GeomUtil.ROTATION2D_ZERO),
        new SwerveModulePosition(0, GeomUtil.ROTATION2D_ZERO)
    };

    @Log.Once
    private final Translation2d[] moduleChassisOffsets = kSwerve.MODULE_CHASSIS_OFFSETS;

    public SlipDetection() {}

    // /**
    //  * Filter the swerve module states to detect slipping and return a better guess of where the robot actually is.
    //  * @param states
    //  * @param positions
    //  * @param acceleration
    //  * @return
    //  */
    // public SwerveModulePosition[] filter(SwerveModuleState[] states, SwerveModulePosition[] positions, double acceleration) {

    // }

    // private boolean modulesStaySquare(SwerveModulePosition[] positions) {
    //     Translation2d[] poses = new Translation2d[4];
    //     for (int i = 0; i < 4; i++) {
    //         var module = modules[i];
    //         var position = positions[i];
    //         double distance = modules[i].distance - positions[i].distanceMeters;
    //         Translation2d movedBy = new Translation2d(distance, position.angle);
    //         poses[i] = module.position.plus(movedBy);
    //     }

        
    // }
}
