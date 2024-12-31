package frc.robot.subsystems.drive;

import static frc.robot.util.subsystemUtils.PhoenixUtil.tryUntilOk;
import static frc.robot.util.subsystemUtils.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.generated.TunerConstants;

public abstract class ModuleIOSparkAndTalon implements ModuleIO {
    protected final SwerveModuleConstants constants;

    protected final TalonFX driveTalon;
    protected final SparkBase turnSpark;
    protected final CANcoder cancoder;

    // Closed loop controllers
    protected final SparkClosedLoopController turnController;

    protected final AbsoluteEncoder turnEncoder;

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    protected final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Inputs from drive motor
    protected final StatusSignal<Angle> drivePosition;
    protected final StatusSignal<AngularVelocity> driveVelocity;
    protected final StatusSignal<Voltage> driveAppliedVolts;
    protected final StatusSignal<Current> driveCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    protected final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    protected final TalonFXConfiguration driveConfig;
    protected final SparkMaxConfig turnConfig;

    protected final Rotation2d zeroRotation;

    protected ModuleIOSparkAndTalon(SwerveModuleConstants constants, int module) {
        this.constants = constants;

        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.CANcoderId, TunerConstants.DrivetrainConstants.CANBusName);

        zeroRotation = switch (module) {
            case 0 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.FrontLeft.CANcoderOffset));
            case 1 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.FrontRight.CANcoderOffset));
            case 2 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.BackLeft.CANcoderOffset));
            case 3 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.BackRight.CANcoderOffset));
            default -> new Rotation2d();};

        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> TunerConstants.FrontLeft.SteerMotorId;
                    case 1 -> TunerConstants.FrontRight.SteerMotorId;
                    case 2 -> TunerConstants.BackLeft.SteerMotorId;
                    case 3 -> TunerConstants.BackRight.SteerMotorId;
                    default -> 0;
                },
                MotorType.kBrushless);

        // Configure drive motor
        driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        turnEncoder = turnSpark.getAbsoluteEncoder();
        turnController = turnSpark.getClosedLoopController();

        turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(TunerConstants.turnInverted)
                .smartCurrentLimit(TunerConstants.turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(TunerConstants.turnEncoderInverted)
                .positionConversionFactor(TunerConstants.turnEncoderPositionFactor)
                .velocityConversionFactor(TunerConstants.turnEncoderVelocityFactor)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(TunerConstants.turnPIDMinInput, TunerConstants.turnPIDMaxInput)
                .pidf(TunerConstants.turnKp, 0.0, TunerConstants.turnKd, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / TunerConstants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.CANcoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.CANcoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(Drive.ODOMETRY_FREQUENCY, drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePositionRad =
                Units.rotationsToRadians(drivePosition.getValueAsDouble()) / constants.DriveMotorGearRatio;
        inputs.driveVelocityRadPerSec =
                Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / constants.DriveMotorGearRatio;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> voltageRequest.withOutput(output);
                    case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
                });
    }

    @Override
    public void setDriveVelocity(double wheelVelocityRadPerSec) {
        double motorVelocityRotPerSec =
                Units.radiansToRotations(wheelVelocityRadPerSec) * constants.DriveMotorGearRatio;
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(motorVelocityRotPerSec);
                    case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(motorVelocityRotPerSec);
                });
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(),
                TunerConstants.turnPIDMinInput,
                TunerConstants.turnPIDMaxInput);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
