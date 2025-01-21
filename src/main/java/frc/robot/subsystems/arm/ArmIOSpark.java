package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;

public class ArmIOSpark implements ArmIO {

    private final MotorIO motor;
    
    public ArmIOSpark() {
        motor = new MotorIOSparkMax(ArmConstants.motorID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        MotorIOInputs motorIOInputs = motor.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }


    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.setVelocity(velocityRadPerSec);
    }

    @Override
    public void stop() {
        motor.stop();
    }
}
