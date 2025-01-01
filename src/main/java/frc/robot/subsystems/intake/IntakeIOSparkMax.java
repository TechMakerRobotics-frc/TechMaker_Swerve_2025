package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeIOSparkMax implements IntakeIO {
    private static final double GEAR_RATIO = 1;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private final DoubleSolenoid solenoid;

    public IntakeIOSparkMax() {
        solenoid = new DoubleSolenoid(
                IntakeConstants.SOLENOID_MODULE_CAN_ID,
                IntakeConstants.SOLENOID_MODULE_TYPE,
                IntakeConstants.SOLENOID_FORWARD_CHANNEL,
                IntakeConstants.SOLENOID_REVERSE_CHANNEL);

        motor = new SparkMax(13, SparkBase.MotorType.kBrushless);

        motor.setCANTimeout(250);

        encoder = motor.getEncoder();

        motorConfig
                .inverted(false)
                .voltageCompensation(12.0)
                .smartCurrentLimit(30)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @param inputs */
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    }

    /** Ativa o solenoide para o estado forward. */
    @Override
    public void extend() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Ativa o solenoide para o estado reverse. */
    @Override
    public void retract() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
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
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {}
}
