// Copyright 2021-2025 FRC 6328
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

import static frc.robot.subsystems.drive.DriveConstants.ModuleConstants.*;
import static frc.robot.util.subsystemUtils.SparkUtil.ifOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;
import frc.robot.util.subsystemUtils.SparkUtil;

import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSparkTalon implements ModuleIO {
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkMax turnSpark;
    private final RelativeEncoder turnEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSparkTalon(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();};
        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> 2;
                    case 1 -> 5;
                    case 2 -> 8;
                    case 3 -> 11;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnEncoder = turnSpark.getEncoder();
        turnController = turnSpark.getClosedLoopController();

        
        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(turnEncoderInverted)
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pidf(0.1, 0.0, 0, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / TunerConstants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        
        // Update turn inputs
        SparkUtil.sparkStickyFault = false;
        ifOk(
                turnSpark,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
       
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnController.setReference(setpoint, ControlType.kPosition);
    }
}
