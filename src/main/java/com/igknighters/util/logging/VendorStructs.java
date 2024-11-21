package com.igknighters.util.logging;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.spns.SpnValue;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.DifferentialControlModeValue;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.FrcLockValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.IsPROLicensedValue;
import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.ctre.phoenix6.signals.Led1OnColorValue;
import com.ctre.phoenix6.signals.Led2OffColorValue;
import com.ctre.phoenix6.signals.Led2OnColorValue;
import com.ctre.phoenix6.signals.Licensing_IsSeasonPassedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.MotorTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.RobotEnableValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.System_StateValue;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;


import edu.wpi.first.util.struct.Struct;
import monologue.ProceduralStructGenerator;

public class VendorStructs {

    public static class CTRE {
        public final static Struct<StatusCode> STATUS_CODE_STRUCT;
        public final static Struct<SpnValue> SPN_VALUE_STRUCT;

        public final static Struct<AbsoluteSensorRangeValue> ABSOLUTE_SENSOR_RANGE_VALUE_STRUCT;
        public final static Struct<AppliedRotorPolarityValue> APPLIED_ROTOR_POLARITY_VALUE_STRUCT;
        public final static Struct<BridgeOutputValue> BRIDGE_OUTPUT_VALUE_STRUCT;
        public final static Struct<ControlModeValue> CONTROL_MODE_VALUE_STRUCT;
        public final static Struct<DeviceEnableValue> DEVICE_ENABLE_VALUE_STRUCT;
        public final static Struct<DifferentialControlModeValue> DIFFERENTIAL_CONTROL_MODE_VALUE_STRUCT;
        public final static Struct<DifferentialSensorSourceValue> DIFFERENTIAL_SENSOR_SOURCE_VALUE_STRUCT;
        public final static Struct<FeedbackSensorSourceValue> FEEDBACK_SENSOR_SOURCE_VALUE_STRUCT;
        public final static Struct<ForwardLimitSourceValue> FORWARD_LIMIT_SOURCE_VALUE_STRUCT;
        public final static Struct<ForwardLimitTypeValue> FORWARD_LIMIT_TYPE_VALUE_STRUCT;
        public final static Struct<FrcLockValue> FRC_LOCK_VALUE_STRUCT;
        public final static Struct<GravityTypeValue> GRAVITY_TYPE_VALUE_STRUCT;
        public final static Struct<InvertedValue> INVERTED_VALUE_STRUCT;
        public final static Struct<IsPROLicensedValue> IS_PRO_LICENSED_VALUE_STRUCT;
        public final static Struct<Led1OffColorValue> LED_1_OFF_COLOR_VALUE_STRUCT;
        public final static Struct<Led1OnColorValue> LED_1_ON_COLOR_VALUE_STRUCT;
        public final static Struct<Led2OffColorValue> LED_2_OFF_COLOR_VALUE_STRUCT;
        public final static Struct<Led2OnColorValue> LED_2_ON_COLOR_VALUE_STRUCT;
        public final static Struct<Licensing_IsSeasonPassedValue> LICENSING_IS_SEASON_PASSED_VALUE_STRUCT;
        public final static Struct<MagnetHealthValue> MAGNET_HEALTH_VALUE_STRUCT;
        public final static Struct<MotionMagicIsRunningValue> MOTION_MAGIC_IS_RUNNING_VALUE_STRUCT;
        public final static Struct<MotorTypeValue> MOTOR_TYPE_VALUE_STRUCT;
        public final static Struct<NeutralModeValue> NEUTRAL_MODE_VALUE_STRUCT;
        public final static Struct<ReverseLimitSourceValue> REVERSE_LIMIT_SOURCE_VALUE_STRUCT;
        public final static Struct<ReverseLimitTypeValue> REVERSE_LIMIT_TYPE_VALUE_STRUCT;
        public final static Struct<ReverseLimitValue> REVERSE_LIMIT_VALUE_STRUCT;
        public final static Struct<RobotEnableValue> ROBOT_ENABLE_VALUE_STRUCT;
        public final static Struct<SensorDirectionValue> SENSOR_DIRECTION_VALUE_STRUCT;
        public final static Struct<StaticFeedforwardSignValue> STATIC_FEEDFORWARD_SIGN_VALUE_STRUCT;
        public final static Struct<System_StateValue> SYSTEM_STATE_VALUE_STRUCT;

        public final static Struct<AudioConfigs> AUDIO_CONFIGS_STRUCT;
        public final static Struct<ClosedLoopGeneralConfigs> CLOSED_LOOP_GENERAL_CONFIGS_STRUCT;
        public final static Struct<ClosedLoopRampsConfigs> CLOSED_LOOP_RAMPS_CONFIGS_STRUCT;
        public final static Struct<CurrentLimitsConfigs> CURRENT_LIMITS_CONFIGS_STRUCT;
        public final static Struct<CustomParamsConfigs> CUSTOM_PARAMS_CONFIGS_STRUCT;
        public final static Struct<DifferentialConstantsConfigs> DIFFERENTIAL_CONSTANTS_CONFIGS_STRUCT;
        public final static Struct<DifferentialSensorsConfigs> DIFFERENTIAL_SENSORS_CONFIGS_STRUCT;
        public final static Struct<FeedbackConfigs> FEEDBACK_CONFIGS_STRUCT;
        public final static Struct<GyroTrimConfigs> GYRO_TRIM_CONFIGS_STRUCT;
        public final static Struct<HardwareLimitSwitchConfigs> HARDWARE_LIMIT_SWITCH_CONFIGS_STRUCT;
        public final static Struct<MagnetSensorConfigs> MAGNET_SENSOR_CONFIGS_STRUCT;
        public final static Struct<MotionMagicConfigs> MOTION_MAGIC_CONFIGS_STRUCT;
        public final static Struct<MotorOutputConfigs> MOTOR_OUTPUT_CONFIGS_STRUCT;
        public final static Struct<MountPoseConfigs> MOUNT_POSE_CONFIGS_STRUCT;
        public final static Struct<OpenLoopRampsConfigs> OPEN_LOOP_RAMPS_CONFIGS_STRUCT;
        public final static Struct<Pigeon2FeaturesConfigs> PIGEON_2_FEATURES_CONFIGS_STRUCT;
        public final static Struct<Slot0Configs> SLOT_0_CONFIGS_STRUCT;
        public final static Struct<Slot1Configs> SLOT_1_CONFIGS_STRUCT;
        public final static Struct<Slot2Configs> SLOT_2_CONFIGS_STRUCT;
        public final static Struct<SoftwareLimitSwitchConfigs> SOFTWARE_LIMIT_SWITCH_CONFIGS_STRUCT;
        public final static Struct<TorqueCurrentConfigs> TORQUE_CURRENT_CONFIGS_STRUCT;
        public final static Struct<VoltageConfigs> VOLTAGE_CONFIGS_STRUCT;
        public final static Struct<TalonFXConfiguration> TALON_FX_CONFIG_STRUCT;
        public final static Struct<CANcoderConfiguration> CAN_CODER_CONFIG_STRUCT;
        public final static Struct<Pigeon2Configuration> PIGEON_2_CONFIG_STRUCT;

        static {
            STATUS_CODE_STRUCT = ProceduralStructGenerator.genEnum(StatusCode.class);
            SPN_VALUE_STRUCT = ProceduralStructGenerator.genEnum(SpnValue.class);

            ABSOLUTE_SENSOR_RANGE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(AbsoluteSensorRangeValue.class);
            APPLIED_ROTOR_POLARITY_VALUE_STRUCT = ProceduralStructGenerator.genEnum(AppliedRotorPolarityValue.class);
            BRIDGE_OUTPUT_VALUE_STRUCT = ProceduralStructGenerator.genEnum(BridgeOutputValue.class);
            CONTROL_MODE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ControlModeValue.class);
            DEVICE_ENABLE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(DeviceEnableValue.class);
            DIFFERENTIAL_CONTROL_MODE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(DifferentialControlModeValue.class);
            DIFFERENTIAL_SENSOR_SOURCE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(DifferentialSensorSourceValue.class);
            FEEDBACK_SENSOR_SOURCE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(FeedbackSensorSourceValue.class);
            FORWARD_LIMIT_SOURCE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ForwardLimitSourceValue.class);
            FORWARD_LIMIT_TYPE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ForwardLimitTypeValue.class);
            FRC_LOCK_VALUE_STRUCT = ProceduralStructGenerator.genEnum(FrcLockValue.class);
            GRAVITY_TYPE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(GravityTypeValue.class);
            INVERTED_VALUE_STRUCT = ProceduralStructGenerator.genEnum(InvertedValue.class);
            IS_PRO_LICENSED_VALUE_STRUCT = ProceduralStructGenerator.genEnum(IsPROLicensedValue.class);
            LED_1_OFF_COLOR_VALUE_STRUCT = ProceduralStructGenerator.genEnum(Led1OffColorValue.class);
            LED_1_ON_COLOR_VALUE_STRUCT = ProceduralStructGenerator.genEnum(Led1OnColorValue.class);
            LED_2_OFF_COLOR_VALUE_STRUCT = ProceduralStructGenerator.genEnum(Led2OffColorValue.class);
            LED_2_ON_COLOR_VALUE_STRUCT = ProceduralStructGenerator.genEnum(Led2OnColorValue.class);
            LICENSING_IS_SEASON_PASSED_VALUE_STRUCT = ProceduralStructGenerator.genEnum(Licensing_IsSeasonPassedValue.class);
            MAGNET_HEALTH_VALUE_STRUCT = ProceduralStructGenerator.genEnum(MagnetHealthValue.class);
            MOTION_MAGIC_IS_RUNNING_VALUE_STRUCT = ProceduralStructGenerator.genEnum(MotionMagicIsRunningValue.class);
            MOTOR_TYPE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(MotorTypeValue.class);
            NEUTRAL_MODE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(NeutralModeValue.class);
            REVERSE_LIMIT_SOURCE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ReverseLimitSourceValue.class);
            REVERSE_LIMIT_TYPE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ReverseLimitTypeValue.class);
            REVERSE_LIMIT_VALUE_STRUCT = ProceduralStructGenerator.genEnum(ReverseLimitValue.class);
            ROBOT_ENABLE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(RobotEnableValue.class);
            SENSOR_DIRECTION_VALUE_STRUCT = ProceduralStructGenerator.genEnum(SensorDirectionValue.class);
            STATIC_FEEDFORWARD_SIGN_VALUE_STRUCT = ProceduralStructGenerator.genEnum(StaticFeedforwardSignValue.class);
            SYSTEM_STATE_VALUE_STRUCT = ProceduralStructGenerator.genEnum(System_StateValue.class);

            AUDIO_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(AudioConfigs.class, AudioConfigs::new);
            CLOSED_LOOP_GENERAL_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(ClosedLoopGeneralConfigs.class, ClosedLoopGeneralConfigs::new);
            CLOSED_LOOP_RAMPS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(ClosedLoopRampsConfigs.class, ClosedLoopRampsConfigs::new);
            CURRENT_LIMITS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(CurrentLimitsConfigs.class, CurrentLimitsConfigs::new);
            CUSTOM_PARAMS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(CustomParamsConfigs.class, CustomParamsConfigs::new);
            DIFFERENTIAL_CONSTANTS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(DifferentialConstantsConfigs.class, DifferentialConstantsConfigs::new);
            DIFFERENTIAL_SENSORS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(DifferentialSensorsConfigs.class, DifferentialSensorsConfigs::new);
            FEEDBACK_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(FeedbackConfigs.class, FeedbackConfigs::new);
            GYRO_TRIM_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(GyroTrimConfigs.class, GyroTrimConfigs::new);
            HARDWARE_LIMIT_SWITCH_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(HardwareLimitSwitchConfigs.class, HardwareLimitSwitchConfigs::new);
            MAGNET_SENSOR_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(MagnetSensorConfigs.class, MagnetSensorConfigs::new);
            MOTION_MAGIC_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(MotionMagicConfigs.class, MotionMagicConfigs::new);
            MOTOR_OUTPUT_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(MotorOutputConfigs.class, MotorOutputConfigs::new);
            MOUNT_POSE_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(MountPoseConfigs.class, MountPoseConfigs::new);
            OPEN_LOOP_RAMPS_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(OpenLoopRampsConfigs.class, OpenLoopRampsConfigs::new);
            PIGEON_2_FEATURES_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(Pigeon2FeaturesConfigs.class, Pigeon2FeaturesConfigs::new);
            SLOT_0_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(Slot0Configs.class, Slot0Configs::new);
            SLOT_1_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(Slot1Configs.class, Slot1Configs::new);
            SLOT_2_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(Slot2Configs.class, Slot2Configs::new);
            SOFTWARE_LIMIT_SWITCH_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(SoftwareLimitSwitchConfigs.class, SoftwareLimitSwitchConfigs::new);
            TORQUE_CURRENT_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(TorqueCurrentConfigs.class, TorqueCurrentConfigs::new);
            VOLTAGE_CONFIGS_STRUCT = ProceduralStructGenerator.genObject(VoltageConfigs.class, VoltageConfigs::new);

            TALON_FX_CONFIG_STRUCT = ProceduralStructGenerator.genObject(TalonFXConfiguration.class, TalonFXConfiguration::new);
            CAN_CODER_CONFIG_STRUCT = ProceduralStructGenerator.genObject(CANcoderConfiguration.class, CANcoderConfiguration::new);
            PIGEON_2_CONFIG_STRUCT = ProceduralStructGenerator.genObject(Pigeon2Configuration.class, Pigeon2Configuration::new);
        }
    }
}
