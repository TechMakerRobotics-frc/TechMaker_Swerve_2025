package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;

public class ModuleIOSparkTalon implements ModuleIO {

  private static final double DRIVE_GEAR_RATIO = ModuleConstants.DRIVE_GEAR_RATIO;
  private static final double TURN_GEAR_RATIO = ModuleConstants.TURN_GEAR_RATIO;

  private final SparkMax turnSparkMax;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final TalonFX driveTalon;

  private boolean isTurnMotorInverted = ModuleConstants.TURN_MOTOR_INVERTED;
  private final Rotation2d absoluteEncoderOffset;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  public ModuleIOSparkTalon(int index) {
    switch (index) {
      case 0: // front left
        driveTalon = new TalonFX(ModuleConstants.TALON_FL, ModuleConstants.CANBUS);
        turnSparkMax = new SparkMax(ModuleConstants.SPARK_FL, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_FL, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_FL);
        break;
      case 1: // front right
        driveTalon = new TalonFX(ModuleConstants.TALON_FR, ModuleConstants.CANBUS);
        turnSparkMax = new SparkMax(ModuleConstants.SPARK_FR, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_FR, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_FR);
        break;
      case 2: // back left
        driveTalon = new TalonFX(ModuleConstants.TALON_BL, ModuleConstants.CANBUS);
        turnSparkMax = new SparkMax(ModuleConstants.SPARK_BL, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_BL, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_BL);
        break;
      case 3: // back right
        driveTalon = new TalonFX(ModuleConstants.TALON_BR, ModuleConstants.CANBUS);
        turnSparkMax = new SparkMax(ModuleConstants.SPARK_BR, MotorType.kBrushless);
        cancoder = new CANcoder(ModuleConstants.CANCODER_BR, ModuleConstants.CANBUS);
        absoluteEncoderOffset = new Rotation2d(ModuleConstants.ENCODER_OFFSET_BR);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Initialize TalonFX configuration
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.SUPPLY_CURRENT_LIMIT;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        ModuleConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(false);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.ODOMETRY_UPDATE_FREQUENCY, drivePosition); // Required for odometry
    BaseStatusSignal.setUpdateFrequencyForAll(
        ModuleConstants.OTHER_SIGNALS_UPDATE_FREQUENCY,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent);

    // Initialize SparkMax configuration
    var turnConfig = new SparkMaxConfig();
    turnSparkMax.setCANTimeout(ModuleConstants.CAN_TIMEOUT_MS);
    
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnConfig.inverted(isTurnMotorInverted);

    turnConfig.smartCurrentLimit(ModuleConstants.SMART_CURRENT_LIMIT);
    turnConfig.voltageCompensation(ModuleConstants.VOLTAGE_COMPENSATION);
    
    turnConfig.alternateEncoder.measurementPeriod(ModuleConstants.ENCODER_MEASUREMENT_PERIOD_MS);
    turnRelativeEncoder.setPosition(ModuleConstants.INITIAL_ENCODER_POSITION);
    turnConfig.alternateEncoder.measurementPeriod(ModuleConstants.ENCODER_MEASUREMENT_PERIOD_MS);
    turnConfig.alternateEncoder.averageDepth(ModuleConstants.ENCODER_AVERAGE_DEPTH);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive (TalonFX)
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveVelocity.getValueAsDouble())
            / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn (CANSparkMax)
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    /*inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);*/
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    turnSparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }
}
