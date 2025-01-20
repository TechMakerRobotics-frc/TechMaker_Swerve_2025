package frc.robot.interfaces.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkMax implements MotorIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController;


    public MotorIOSparkMax(int id, SparkBase.MotorType type, boolean inverted,int timeout, double voltageCompensation, int smartCurrentLimit, SparkBaseConfig.IdleMode idleMode) {
        motor = new SparkMax(id, type);

        motor.setCANTimeout(timeout);

        encoder = motor.getEncoder();
        

        motorConfig
                .inverted(inverted)
                .voltageCompensation(voltageCompensation)
                .smartCurrentLimit(smartCurrentLimit)
                .idleMode(idleMode);
        motorConfig.encoder
                .positionConversionFactor(2.0/42.0)
                .velocityConversionFactor(1);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(10)
                .i(0)
                .d(0.1)
                .outputRange(-1, 1);
                // Set PID values for velocity control in slot 1
                //.p(0.0001, ClosedLoopSlot.kSlot1)
                //.i(0, ClosedLoopSlot.kSlot1)
                //.d(0, ClosedLoopSlot.kSlot1)
                //.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                //.outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        
     


        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopController = motor.getClosedLoopController();
    }

    /** @param inputs */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRot = encoder.getPosition() ;
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() );
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
        
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.set(velocityRadPerSec);
    }
    @Override
    public void setPosition(double position) {
        closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    }
    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        motorConfig.closedLoop.pid(kP, kI, kD);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void configurePIDF(double kP, double kI, double kD, double kF) {
        motorConfig.closedLoop.pidf(kP, kI, kD, kF);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void resetPosition() {
        encoder.setPosition(0);
    }
    @Override
    public void setOffset(double offset) {
        encoder.setPosition(offset);
    }
}
