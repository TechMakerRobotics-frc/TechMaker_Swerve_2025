package frc.robot.subsystems.drive;

import static frc.robot.util.subsystemUtils.SparkUtil.ifOk;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.subsystemUtils.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Talon FX drive motor controller, SparkMax turn motor controller and CANcoder. Configured
 * using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOSparkAndTalonFXReal extends ModuleIOSparkAndTalon {

    // Queue to read inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    public ModuleIOSparkAndTalonFXReal(SwerveModuleConstants<ParentConfiguration,ParentConfiguration,ParentConfiguration> constants, int module) {
        super(constants, module);

        this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);
        this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble(Units::rotationsToRadians)
                .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

        // Update turn inputs
        SparkUtil.sparkStickyFault = false;
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnAbsolutePosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        this.turnConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        turnSpark.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        driveConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(driveConfig);
    }
}
