// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class motor extends SubsystemBase {
  private final MotorIOSparkMax motor1 = new MotorIOSparkMax(2, SparkBase.MotorType.kBrushless, false, 250, 10.0, 30, SparkBaseConfig.IdleMode.kCoast);
  private final MotorIOSparkMax motor2 = new MotorIOSparkMax(5, SparkBase.MotorType.kBrushless, false, 250, 10.0, 30, SparkBaseConfig.IdleMode.kCoast);
  private final MotorIOSparkMax motor3 = new MotorIOSparkMax(8, SparkBase.MotorType.kBrushless, false, 250, 10.0, 30, SparkBaseConfig.IdleMode.kCoast);
  private final MotorIOSparkMax motor4 = new MotorIOSparkMax(11, SparkBase.MotorType.kBrushless, false, 250, 10.0, 30, SparkBaseConfig.IdleMode.kCoast);
  private MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  /** Creates a new motor. */
  public motor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor1.updateInputs(inputs);
    Logger.processInputs("motor 1"  , inputs);
    motor2.updateInputs(inputs);
    Logger.processInputs("motor 2"  , inputs);
    motor3.updateInputs(inputs);
    Logger.processInputs("motor 3"  , inputs);
    motor4.updateInputs(inputs);
    Logger.processInputs("motor 4"  , inputs);
  }
  public void setPosition(double positionRad) {
    motor1.setPosition(positionRad);
    motor2.setPosition(positionRad);
    motor3.setPosition(positionRad);
    motor4.setPosition(positionRad);
  }
}
