package frc.robot.subsystems.flywheel;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class FlywheelIOSparkMax implements FlywheelIO {
    private static final double GEAR_RATIO = 1;

    private final SparkMax upMotor;
    private final SparkMax downMotor;
    private final SparkMaxConfig upMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig downMotorConfig = new SparkMaxConfig();

    public FlywheelIOSparkMax() {
        upMotor = new SparkMax(15, SparkBase.MotorType.kBrushless);
        downMotor = new SparkMax(16, SparkBase.MotorType.kBrushless);

        upMotor.setCANTimeout(250);
        downMotor.setCANTimeout(250);

        upMotorConfig
                .inverted(false)
                .voltageCompensation(12.0)
                .smartCurrentLimit(30)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        upMotor.configure(upMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        downMotorConfig
                .inverted(false)
                .voltageCompensation(12)
                .smartCurrentLimit(30)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        downMotor.configure(downMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @param inputs */
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(GEAR_RATIO);
        inputs.appliedVolts = upMotor.getAppliedOutput() * upMotor.getBusVoltage();
        inputs.currentAmps = new double[] {upMotor.getOutputCurrent(), downMotor.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double volts) {
        upMotor.setVoltage(volts);
        downMotor.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        upMotor.set(velocityRadPerSec);
        downMotor.set(velocityRadPerSec);
    }

    @Override
    public void stop() {
        upMotor.stopMotor();
        downMotor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {}
}
