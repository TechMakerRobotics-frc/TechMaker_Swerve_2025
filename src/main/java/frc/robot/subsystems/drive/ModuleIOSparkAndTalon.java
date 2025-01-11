package frc.robot.subsystems.drive;

import static frc.robot.util.subsystemUtils.PhoenixUtil.tryUntilOk;
import static frc.robot.util.subsystemUtils.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
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
import com.revrobotics.spark.ClosedLoopSlot;
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
    protected final SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> constants;

    protected final TalonFX driveTalon;
    protected final SparkMax turnSpark;
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

    protected ModuleIOSparkAndTalon(SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> constants, int module) {
        this.constants = constants;

        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        zeroRotation = switch (module) {
            case 0 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.kFrontLeftEncoderOffsetRotations));
            case 1 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.kFrontRightEncoderOffsetRotations));
            case 2 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.kBackLeftEncoderOffsetRotations));
            case 3 -> new Rotation2d(Units.rotationsToRadians(TunerConstants.kBackRightEncoderOffsetRotations));
            default -> new Rotation2d();};

        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> TunerConstants.kFrontLeftSteerMotorId;
                    case 1 -> TunerConstants.kFrontRightSteerMotorId;
                    case 2 -> TunerConstants.kBackLeftSteerMotorId;
                    case 3 -> TunerConstants.kBackRightSteerMotorId;
                    default -> 0;
                },
                MotorType.kBrushless);

        // Configure drive motor
        driveConfig = new TalonFXConfiguration();
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
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(TunerConstants.turnKp)
                .i(0)
                .d(TunerConstants.turnKd)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(TunerConstants.turnKp, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(TunerConstants.turnKd, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
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
