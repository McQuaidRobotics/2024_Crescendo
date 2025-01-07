package igknighters.util.logging;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import com.ctre.phoenix6.spns.SpnValue;
import edu.wpi.first.util.struct.Struct;
import monologue.ProceduralStructGenerator;

public class VendorStructs {

  public static final Struct<StatusCode> STATUS_CODE_STRUCT =
      ProceduralStructGenerator.genEnum(StatusCode.class);
  public static final Struct<SpnValue> SPN_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(SpnValue.class);

  public static final Struct<AbsoluteSensorRangeValue> ABSOLUTE_SENSOR_RANGE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(AbsoluteSensorRangeValue.class);
  public static final Struct<AppliedRotorPolarityValue> APPLIED_ROTOR_POLARITY_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(AppliedRotorPolarityValue.class);
  public static final Struct<BridgeOutputValue> BRIDGE_OUTPUT_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(BridgeOutputValue.class);
  public static final Struct<ControlModeValue> CONTROL_MODE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ControlModeValue.class);
  public static final Struct<DeviceEnableValue> DEVICE_ENABLE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(DeviceEnableValue.class);
  public static final Struct<DifferentialControlModeValue> DIFFERENTIAL_CONTROL_MODE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(DifferentialControlModeValue.class);
  public static final Struct<DifferentialSensorSourceValue>
      DIFFERENTIAL_SENSOR_SOURCE_VALUE_STRUCT =
          ProceduralStructGenerator.genEnum(DifferentialSensorSourceValue.class);
  public static final Struct<FeedbackSensorSourceValue> FEEDBACK_SENSOR_SOURCE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(FeedbackSensorSourceValue.class);
  public static final Struct<ForwardLimitSourceValue> FORWARD_LIMIT_SOURCE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ForwardLimitSourceValue.class);
  public static final Struct<ForwardLimitTypeValue> FORWARD_LIMIT_TYPE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ForwardLimitTypeValue.class);
  public static final Struct<FrcLockValue> FRC_LOCK_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(FrcLockValue.class);
  public static final Struct<GravityTypeValue> GRAVITY_TYPE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(GravityTypeValue.class);
  public static final Struct<InvertedValue> INVERTED_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(InvertedValue.class);
  public static final Struct<IsPROLicensedValue> IS_PRO_LICENSED_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(IsPROLicensedValue.class);
  public static final Struct<Led1OffColorValue> LED_1_OFF_COLOR_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(Led1OffColorValue.class);
  public static final Struct<Led1OnColorValue> LED_1_ON_COLOR_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(Led1OnColorValue.class);
  public static final Struct<Led2OffColorValue> LED_2_OFF_COLOR_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(Led2OffColorValue.class);
  public static final Struct<Led2OnColorValue> LED_2_ON_COLOR_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(Led2OnColorValue.class);
  public static final Struct<Licensing_IsSeasonPassedValue>
      LICENSING_IS_SEASON_PASSED_VALUE_STRUCT =
          ProceduralStructGenerator.genEnum(Licensing_IsSeasonPassedValue.class);
  public static final Struct<MagnetHealthValue> MAGNET_HEALTH_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(MagnetHealthValue.class);
  public static final Struct<MotionMagicIsRunningValue> MOTION_MAGIC_IS_RUNNING_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(MotionMagicIsRunningValue.class);
  public static final Struct<MotorTypeValue> MOTOR_TYPE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(MotorTypeValue.class);
  public static final Struct<NeutralModeValue> NEUTRAL_MODE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(NeutralModeValue.class);
  public static final Struct<ReverseLimitSourceValue> REVERSE_LIMIT_SOURCE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ReverseLimitSourceValue.class);
  public static final Struct<ReverseLimitTypeValue> REVERSE_LIMIT_TYPE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ReverseLimitTypeValue.class);
  public static final Struct<ReverseLimitValue> REVERSE_LIMIT_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(ReverseLimitValue.class);
  public static final Struct<RobotEnableValue> ROBOT_ENABLE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(RobotEnableValue.class);
  public static final Struct<SensorDirectionValue> SENSOR_DIRECTION_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(SensorDirectionValue.class);
  public static final Struct<StaticFeedforwardSignValue> STATIC_FEEDFORWARD_SIGN_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(StaticFeedforwardSignValue.class);
  public static final Struct<System_StateValue> SYSTEM_STATE_VALUE_STRUCT =
      ProceduralStructGenerator.genEnum(System_StateValue.class);

  public static final Struct<AudioConfigs> AUDIO_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(AudioConfigs.class, AudioConfigs::new);
  public static final Struct<ClosedLoopGeneralConfigs> CLOSED_LOOP_GENERAL_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          ClosedLoopGeneralConfigs.class, ClosedLoopGeneralConfigs::new);
  public static final Struct<ClosedLoopRampsConfigs> CLOSED_LOOP_RAMPS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          ClosedLoopRampsConfigs.class, ClosedLoopRampsConfigs::new);
  public static final Struct<CurrentLimitsConfigs> CURRENT_LIMITS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(CurrentLimitsConfigs.class, CurrentLimitsConfigs::new);
  public static final Struct<CustomParamsConfigs> CUSTOM_PARAMS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(CustomParamsConfigs.class, CustomParamsConfigs::new);
  public static final Struct<DifferentialConstantsConfigs> DIFFERENTIAL_CONSTANTS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          DifferentialConstantsConfigs.class, DifferentialConstantsConfigs::new);
  public static final Struct<DifferentialSensorsConfigs> DIFFERENTIAL_SENSORS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          DifferentialSensorsConfigs.class, DifferentialSensorsConfigs::new);
  public static final Struct<FeedbackConfigs> FEEDBACK_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(FeedbackConfigs.class, FeedbackConfigs::new);
  public static final Struct<GyroTrimConfigs> GYRO_TRIM_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(GyroTrimConfigs.class, GyroTrimConfigs::new);
  public static final Struct<HardwareLimitSwitchConfigs> HARDWARE_LIMIT_SWITCH_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          HardwareLimitSwitchConfigs.class, HardwareLimitSwitchConfigs::new);
  public static final Struct<MagnetSensorConfigs> MAGNET_SENSOR_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(MagnetSensorConfigs.class, MagnetSensorConfigs::new);
  public static final Struct<MotionMagicConfigs> MOTION_MAGIC_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(MotionMagicConfigs.class, MotionMagicConfigs::new);
  public static final Struct<MotorOutputConfigs> MOTOR_OUTPUT_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(MotorOutputConfigs.class, MotorOutputConfigs::new);
  public static final Struct<MountPoseConfigs> MOUNT_POSE_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(MountPoseConfigs.class, MountPoseConfigs::new);
  public static final Struct<OpenLoopRampsConfigs> OPEN_LOOP_RAMPS_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(OpenLoopRampsConfigs.class, OpenLoopRampsConfigs::new);
  public static final Struct<Pigeon2FeaturesConfigs> PIGEON_2_FEATURES_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          Pigeon2FeaturesConfigs.class, Pigeon2FeaturesConfigs::new);
  public static final Struct<Slot0Configs> SLOT_0_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(Slot0Configs.class, Slot0Configs::new);
  public static final Struct<Slot1Configs> SLOT_1_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(Slot1Configs.class, Slot1Configs::new);
  public static final Struct<Slot2Configs> SLOT_2_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(Slot2Configs.class, Slot2Configs::new);
  public static final Struct<SoftwareLimitSwitchConfigs> SOFTWARE_LIMIT_SWITCH_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(
          SoftwareLimitSwitchConfigs.class, SoftwareLimitSwitchConfigs::new);
  public static final Struct<TorqueCurrentConfigs> TORQUE_CURRENT_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(TorqueCurrentConfigs.class, TorqueCurrentConfigs::new);
  public static final Struct<VoltageConfigs> VOLTAGE_CONFIGS_STRUCT =
      ProceduralStructGenerator.genObject(VoltageConfigs.class, VoltageConfigs::new);
  public static final Struct<TalonFXConfiguration> TALON_FX_CONFIG_STRUCT =
      ProceduralStructGenerator.genObject(TalonFXConfiguration.class, TalonFXConfiguration::new);
  public static final Struct<CANcoderConfiguration> CAN_CODER_CONFIG_STRUCT =
      ProceduralStructGenerator.genObject(CANcoderConfiguration.class, CANcoderConfiguration::new);
  public static final Struct<Pigeon2Configuration> PIGEON_2_CONFIG_STRUCT =
      ProceduralStructGenerator.genObject(Pigeon2Configuration.class, Pigeon2Configuration::new);
}
