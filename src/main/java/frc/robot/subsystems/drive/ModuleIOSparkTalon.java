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

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.interfaces.motor.MotorIOTalonFX;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;


/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSparkTalon implements ModuleIO {

    private final MotorIO driveIO;
    private final MotorIO turnIO;

    public ModuleIOSparkTalon(int module) { 
        switch (module) {
                case 0:  
                        driveIO = new MotorIOTalonFX(1, CANBUS, NeutralModeValue.Coast, DRIVE_GAINS, TunerConstants.kSlipCurrentDouble, SUPPLY_CURRENT_LIMIT_ENABLE, InvertedValue.CounterClockwise_Positive, TunerConstants.kDriveClosedLoopOutput);
                        turnIO = new EncoderIOThroughBore(2, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_FL));
                break;

                case 1: 
                        driveIO = new MotorIOTalonFX(4, CANBUS, NeutralModeValue.Coast, DRIVE_GAINS, TunerConstants.kSlipCurrentDouble, SUPPLY_CURRENT_LIMIT_ENABLE, InvertedValue.CounterClockwise_Positive, TunerConstants.kDriveClosedLoopOutput);
                        turnIO = new EncoderIOThroughBore(5, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_FR));
                break;
                
                case 2: 
                        driveIO = new MotorIOTalonFX(7, CANBUS, NeutralModeValue.Coast, DRIVE_GAINS, TunerConstants.kSlipCurrentDouble, SUPPLY_CURRENT_LIMIT_ENABLE, InvertedValue.CounterClockwise_Positive, TunerConstants.kDriveClosedLoopOutput);
                        turnIO = new EncoderIOThroughBore(8, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_BL));
                break;

                case 3: 
                        driveIO = new MotorIOTalonFX(10, CANBUS, NeutralModeValue.Coast, DRIVE_GAINS, TunerConstants.kSlipCurrentDouble, SUPPLY_CURRENT_LIMIT_ENABLE, InvertedValue.CounterClockwise_Positive, TunerConstants.kDriveClosedLoopOutput);
                        turnIO = new EncoderIOThroughBore(11, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kCoast);
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_BR));
                break;

                default:
                        driveIO = new MotorIO() {}; 
                        turnIO = new MotorIO() {};
                break;
        }
    }

    @Override 
    public void setVoltage(double volts) {
        turnIO.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        turnIO.setVelocity(velocityRadPerSec);
    }
    
    @Override
    public void setPosition(double positionRot) {
        turnIO.setPosition(positionRot);
    }
    
    @Override
    public void stop() {
        turnIO.stop();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveIO.setVelocity(velocityRadPerSec);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        MotorIOInputs motorIOInputs = turnIO.getMotorIOInputs();
        inputs.turnAppliedVolts = motorIOInputs.appliedVolts;
        inputs.turnConnected = motorIOInputs.appliedVolts != 0.0;
        inputs.turnCurrentAmps = motorIOInputs.currentAmps[0];
        inputs.turnPosition = new Rotation2d(Units.rotationsToRadians(motorIOInputs.positionRot));
        inputs.turnVelocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.turnPositionRot = Units.rotationsToRadians(motorIOInputs.positionRot);
    }
}
