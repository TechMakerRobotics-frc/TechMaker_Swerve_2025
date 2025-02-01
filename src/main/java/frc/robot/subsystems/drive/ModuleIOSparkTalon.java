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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.interfaces.motor.MotorIOTalonFX;

import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSparkTalon implements ModuleIO {

  private final MotorIO driveIO;
  private final MotorIO turnIO;
  private CANcoder cancoder;
  private double offset;

  public ModuleIOSparkTalon(int module) {
    switch (module) {
      case 0:
        driveIO =
            new MotorIOTalonFX(
                TALON_FL,
                CANBUS,
                NeutralModeValue.Coast,
                DRIVE_GAINS,
                TunerConstants.kSlipCurrentDouble,
                SUPPLY_CURRENT_LIMIT_ENABLE,
                InvertedValue.Clockwise_Positive,
                TunerConstants.kDriveClosedLoopOutput);
        turnIO =
            new MotorIOSparkMax(
                SPARK_FL, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
        cancoder = new CANcoder(CANCODER_FL, CANBUS);
        offset = ENCODER_OFFSET_FL;

        break;

      case 1:
        driveIO =
            new MotorIOTalonFX(
                TALON_FR,
                CANBUS,
                NeutralModeValue.Coast,
                DRIVE_GAINS,
                TunerConstants.kSlipCurrentDouble,
                SUPPLY_CURRENT_LIMIT_ENABLE,
                InvertedValue.CounterClockwise_Positive,
                TunerConstants.kDriveClosedLoopOutput);
        turnIO =
            new MotorIOSparkMax(
                SPARK_FR, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
        cancoder = new CANcoder(CANCODER_FR, CANBUS);
        offset = ENCODER_OFFSET_FR;
        break;

      case 2:
        driveIO =
            new MotorIOTalonFX(
                TALON_BL,
                CANBUS,
                NeutralModeValue.Coast,
                DRIVE_GAINS,
                TunerConstants.kSlipCurrentDouble,
                SUPPLY_CURRENT_LIMIT_ENABLE,
                InvertedValue.Clockwise_Positive,
                TunerConstants.kDriveClosedLoopOutput);
        turnIO =
            new MotorIOSparkMax(
                SPARK_BL, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
        cancoder = new CANcoder(CANCODER_BL, CANBUS);
        offset = ENCODER_OFFSET_BL;
        break;

      case 3:
        driveIO =
            new MotorIOTalonFX(
                TALON_BR,
                CANBUS,
                NeutralModeValue.Coast,
                DRIVE_GAINS,
                TunerConstants.kSlipCurrentDouble,
                SUPPLY_CURRENT_LIMIT_ENABLE,
                InvertedValue.CounterClockwise_Positive,
                TunerConstants.kDriveClosedLoopOutput);
        turnIO =
            new MotorIOSparkMax(
                SPARK_BR, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
        cancoder = new CANcoder(CANCODER_BR, CANBUS);
        offset = ENCODER_OFFSET_BR;
        break;

      default:
        driveIO = new MotorIO() {};
        turnIO = new MotorIO() {};
        offset = 0.0;
        break;
    }
    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnIO.setOffset(cancoder.getAbsolutePosition().getValueAsDouble() - offset);
    turnIO.setPosition(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    MotorIOInputs motorIOInputs = turnIO.getMotorIOInputs();
    inputs.turnOffset = Units.rotationsToRadians(offset);
    inputs.turnAppliedVolts = motorIOInputs.appliedVolts;
    inputs.turnConnected = motorIOInputs.appliedVolts != 0.0;
    inputs.turnCurrentAmps = motorIOInputs.currentAmps[0];
    inputs.turnPosition = new Rotation2d(Units.rotationsToRadians(motorIOInputs.positionRot));
    inputs.turnVelocityRadPerSec = motorIOInputs.velocityRadPerSec / TURN_GEAR_RATIO;
    inputs.turnPositionRot = motorIOInputs.positionRot;
    motorIOInputs = driveIO.getMotorIOInputs();
    inputs.driveAppliedVolts = motorIOInputs.appliedVolts;
    inputs.driveConnected = motorIOInputs.appliedVolts != 0.0;
    inputs.driveCurrentAmps = motorIOInputs.currentAmps[0];
    inputs.drivePositionRad = Units.rotationsToRadians(motorIOInputs.positionRot) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec = motorIOInputs.velocityRadPerSec / DRIVE_GEAR_RATIO;
  }

  @Override
  public void runDriveVelocity(double velocityRadPerSec) {
    driveIO.setVelocity(velocityRadPerSec);
  }

  @Override
  public void runTurnPosition(Rotation2d positionRot) {
    Logger.recordOutput("runTurnPosition - positionRot", positionRot);
    double setpoint = positionRot.getRotations();

    Logger.recordOutput("runTurnPosition - setpoint", setpoint);

    turnIO.setPosition(setpoint);
  }

  @Override
  public void runDriveOpenLoop(double output) {
    driveIO.setVoltage(output);
  }

  @Override
  public void runTurnOpenLoop(double output) {
    turnIO.setVoltage(output);
  }

  @Override
  public void stop() {
    driveIO.stop();
    turnIO.stop();
  }
}
