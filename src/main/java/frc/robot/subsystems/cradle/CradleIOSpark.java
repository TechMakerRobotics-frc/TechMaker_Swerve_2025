package frc.robot.subsystems.cradle;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIOSparkMax;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;

public class CradleIOSpark implements CradleIO {

    private final MotorIO alfa;
    private final MotorIO beta;
    private final MotorIO gama;
    
    public CradleIOSpark() {
        alfa = new MotorIOSparkMax(CradleConstants.ALFA_ID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
        beta = new MotorIOSparkMax(CradleConstants.BETA_ID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
        gama = new MotorIOSparkMax(CradleConstants.GAMA_ID, MotorType.kBrushless, true, 250, 10.0, 30, IdleMode.kBrake);
    }

    @Override
    public void updateInputs(CradleIOInputs inputs) {
        MotorIOInputs motorIOInputs = alfa.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);

        motorIOInputs = beta.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);

        motorIOInputs = gama.getMotorIOInputs();
        inputs.appliedVolts = motorIOInputs.appliedVolts;
        inputs.currentAmps = motorIOInputs.currentAmps[0];
        inputs.velocityRadPerSec = motorIOInputs.velocityRadPerSec;
        inputs.positionRad = Units.rotationsToRadians(motorIOInputs.positionRot);
    }

    @Override
    public void setVoltageForAll(double volts) {
        alfa.setVoltage(volts);
        beta.setVoltage(volts);
        gama.setVoltage(volts);
    }

    @Override
    public void setVoltageAlfaMotor(double volts) {
        alfa.setVoltage(volts);
    }
    
    @Override
    public void setVoltageBetaMotor(double volts) {
        beta.setVoltage(volts);
    }

    @Override
    public void setVoltageGamaMotor(double volts) {
        gama.setVoltage(volts);
    }

    @Override
    public void setVelocityForAll(double velocityRadPerSec) {
        alfa.setVelocity(velocityRadPerSec);
        beta.setVelocity(velocityRadPerSec);
        gama.setVelocity(velocityRadPerSec);
    }

    @Override
    public void setVelocityAlfaMotor(double velocityRadPerSec) {
        alfa.setVelocity(velocityRadPerSec);
    }

    @Override
    public void setVelocityBetaMotor(double velocityRadPerSec) {
        beta.setVelocity(velocityRadPerSec);
    }
    
    @Override
    public void setVelocityGamaMotor(double velocityRadPerSec) {
        gama.setVelocity(velocityRadPerSec);
    }

    @Override
    public void stopAll() {
        alfa.stop();
        beta.stop();
        gama.stop();
    }

    @Override
    public void stopAlfa() {
        alfa.stop();
    }
    
    @Override
    public void stopBeta() {
        beta.stop();
    }
    
    @Override
    public void stopGama() {
        gama.stop();
    }
}
