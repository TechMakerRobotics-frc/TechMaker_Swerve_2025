// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.util.subsystemUtils.SparkUtil.ifOk;
import static frc.robot.util.subsystemUtils.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.subsystemUtils.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and CANcoder.
 * Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
    // Queue to read inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase turnSpark;
    private final AbsoluteEncoder turnEncoder;

    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOTalonFXReal(SwerveModuleConstants constants, int module) {
        super(constants);
        zeroRotation = switch (module) {
            case 0 -> DriveConstants.frontLeftZeroRotation;
            case 1 -> DriveConstants.frontRightZeroRotation;
            case 2 -> DriveConstants.backLeftZeroRotation;
            case 3 -> DriveConstants.backRightZeroRotation;
            default -> new Rotation2d();};

        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> DriveConstants.frontLeftTurnCanId;
                    case 1 -> DriveConstants.frontRightTurnCanId;
                    case 2 -> DriveConstants.backLeftTurnCanId;
                    case 3 -> DriveConstants.backRightTurnCanId;
                    default -> 0;
                },
                MotorType.kBrushless);

        turnEncoder = turnSpark.getAbsoluteEncoder();
        turnController = turnSpark.getClosedLoopController();

        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(DriveConstants.turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(DriveConstants.turnEncoderInverted)
                .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
                .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
                .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);

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
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.plus(zeroRotation).getRadians(),
                DriveConstants.turnPIDMinInput,
                DriveConstants.turnPIDMaxInput);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
