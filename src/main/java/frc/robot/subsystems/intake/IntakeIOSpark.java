package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;

public class IntakeIOSpark implements IntakeIO {

    private final MotorIO intakeMotor;
    private final MotorIO rollerMotor;
    
    public IntakeIOSpark() {
        intakeMotor = new MotorIOSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
        rollerMotor = new MotorIOSparkMax(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        MotorIOInputs motorIOInputs = intakeMotor.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);

        motorIOInputs = rollerMotor.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
    }

    @Override
    public void setVoltageIntakeMotor(double volts) {
        intakeMotor.setVoltage(volts);
    }
    
    @Override
    public void setVoltageRollerMotor(double volts) {
        rollerMotor.setVoltage(volts);
    }

    @Override
    public void setVelocityIntakeMotor(double velocityRadPerSec) {
        intakeMotor.setVelocity(velocityRadPerSec);
    }

    @Override
    public void setVelocityRollerMotor(double velocityRadPerSec) {
        rollerMotor.setVelocity(velocityRadPerSec);
    }

    @Override
    public void stopAll() {
        intakeMotor.stop();
        rollerMotor.stop();
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }
    
    @Override
    public void stopRollerMotor() {
        rollerMotor.stop();
    }
}
