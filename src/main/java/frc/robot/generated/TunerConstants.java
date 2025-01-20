package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class TunerConstants {

    public static final double maxSpeedMetersPerSec = 54.8;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Front Left
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontLeftEncoderId = 3;
    public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.006104);
    public static final double kFrontLeftEncoderOffsetRotations = -0.006104;
    public static final boolean kFrontLeftSteerMotorInverted = turnInverted;
    public static final boolean kFrontLeftCANcoderInverted = turnEncoderInverted;

    public static final Distance kFrontLeftXPos = Inches.of(10.5);
    public static final Distance kFrontLeftYPos = Inches.of(10.5);

    // Front Right
    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 5;
    public static final int kFrontRightEncoderId = 6;
    public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.001465);
    public static final double kFrontRightEncoderOffsetRotations = -0.001465;
    public static final boolean kFrontRightSteerMotorInverted = turnInverted;
    public static final boolean kFrontRightCANcoderInverted = turnEncoderInverted;

    public static final Distance kFrontRightXPos = Inches.of(10.5);
    public static final Distance kFrontRightYPos = Inches.of(-10.5);

    // Back Left
    public static final int kBackLeftDriveMotorId = 7;
    public static final int kBackLeftSteerMotorId = 8;
    public static final int kBackLeftEncoderId = 9;
    public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.001953);
    public static final double kBackLeftEncoderOffsetRotations = -0.001953;
    public static final boolean kBackLeftSteerMotorInverted = turnInverted;
    public static final boolean kBackLeftCANcoderInverted = turnEncoderInverted;

    public static final Distance kBackLeftXPos = Inches.of(-10.5);
    public static final Distance kBackLeftYPos = Inches.of(10.5);

    // Back Right
    public static final int kBackRightDriveMotorId = 10;
    public static final int kBackRightSteerMotorId = 11;
    public static final int kBackRightEncoderId = 12;
    public static final Angle kBackRightEncoderOffset = Rotations.of(-0.375732);
    public static final double kBackRightEncoderOffsetRotations = -0.375732;
    public static final boolean kBackRightSteerMotorInverted = turnInverted;
    public static final boolean kBackRightCANcoderInverted = turnEncoderInverted;

    public static final Distance kBackRightXPos = Inches.of(-10.5);
    public static final Distance kBackRightYPos = Inches.of(-10.5);

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(2.0)
            .withKS(0.2)
            .withKV(1.59)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains =
            new Slot0Configs().withKP(0.03).withKI(0).withKD(0).withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
    // RemoteCANcoder
    public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final Current kSlipCurrent = Amps.of(120.0);
    public static final Double kSlipCurrentDouble = 120.0;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot
    // be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));
    public static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(maxSpeedMetersPerSec);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.5;

    public static final double kDriveGearRatio = 7.363636364;
    public static final double kSteerGearRatio = 12.8;
    public static final Distance kWheelRadius = Inches.of(2.167);

    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final int kPigeonId = 14;

    // These are only used for simulation
    public static final double kSteerInertia = 0.01;
    public static final double kDriveInertia = 0.025;
    // Simulated voltage necessary to overcome friction
    public static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
    public static final Voltage kDriveFrictionVoltage = Volts.of(0.25);

    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 60;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction = (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14
    // pinion teeth and 22
    // spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor
    // Rotations
    // -> Wheel
    // Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor
    // RPM
    // ->
    // Wheel
    // Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 0.1;
    public static final double turnKd = 0.0;
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 45;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1),
            moduleTranslations);

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(() -> new SwerveModuleSimulation(
                new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF)));

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

             private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator = new SwerveModuleConstantsFactory<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(cancoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);
        
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft = ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftCANcoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightCANcoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftCANcoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightCANcoderInverted);

                    private static final SwerveModuleConstantsFactory<ParentConfiguration,ParentConfiguration,ParentConfiguration> ConstantCreatorSIM = new SwerveModuleConstantsFactory<>()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(kWheelRadius)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kSpeedAt12Volts)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(cancoderInitialConfigs)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage);
            public static final SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> FrontLeftSIM = ConstantCreatorSIM.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftCANcoderInverted);
            public static final SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> FrontRightSIM = ConstantCreatorSIM.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightCANcoderInverted);
            public static final SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> BackLeftSIM = ConstantCreatorSIM.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftCANcoderInverted);
            public static final SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> BackRightSIM = ConstantCreatorSIM.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightCANcoderInverted);
}