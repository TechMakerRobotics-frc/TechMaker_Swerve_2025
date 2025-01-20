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

import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.Motor.MotorIO;
import frc.robot.interfaces.Motor.MotorIO.MotorIOInputs;


/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSparkTalon implements ModuleIO {

    private final MotorIO turnIO;

    public ModuleIOSparkTalon(int module, MotorIO turnMotorIO) { 
        switch (module) {
                case 0:  turnIO = turnMotorIO;
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_FL));
                break;

                case 1: turnIO = turnMotorIO;
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_FR));
                break;
                
                case 2: turnIO = turnMotorIO;
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_BL));
                break;

                case 3: turnIO = turnMotorIO;
                        turnIO.setOffset(Units.radiansToRotations(ENCODER_OFFSET_BR));
                break;

                default: turnIO = new MotorIO() {};
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
    public void updateInputs(ModuleIOInputs inputs) {
        MotorIOInputs motorIOInputs = new MotorIOInputs();
        motorIOInputs.appliedVolts = inputs.turnAppliedVolts;
        motorIOInputs.currentAmps = new double[] {inputs.turnCurrentAmps};
        motorIOInputs.positionRot = inputs.turnPositionRot;
        motorIOInputs.velocityRadPerSec = inputs.turnVelocityRadPerSec;
        turnIO.updateInputs(motorIOInputs);
    }
}
