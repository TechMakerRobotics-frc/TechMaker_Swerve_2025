package frc.robot.subsystems.lockwheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;

public class LockwheelIOVictorSPX implements LockwheelIO {
    private static final int ENCODER_PULSES_PER_REV = 2048; // Ajuste de acordo com o encoder usado

    private final WPI_VictorSPX motor = new WPI_VictorSPX(15);

    private double currentVelocityRotations = 0;
    private double appliedVolts = 0;
    private double motorCurrent = 0;

    private final Encoder encoder = new Encoder(2, 3);

    public LockwheelIOVictorSPX() {
        motor.configFactoryDefault();

        motor.setNeutralMode(NeutralMode.Coast);

        motor.configVoltageCompSaturation(12.0);
        motor.enableVoltageCompensation(true);

        motor.configOpenloopRamp(0.1); // Slope for ramping up voltage
        encoder.reset();
    }

    @Override
    public void updateInputs(LockwheelIOInputs inputs) {
        // Calcular a velocidade em rotações por minuto (RPM)
        currentVelocityRotations = (encoder.getRate() / ENCODER_PULSES_PER_REV) * 60.0;

        // Atualizar os inputs
        inputs.velocityRadPerSec = Units.rotationsToRadians(currentVelocityRotations);
        inputs.positionRad = Units.rotationsToRadians((double) encoder.get() / ENCODER_PULSES_PER_REV);
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {motorCurrent};
    }

    @Override
    public void setVoltage(double volts) {
        motor.set(ControlMode.PercentOutput, volts / 12.0);
        appliedVolts = volts;
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motor.set(ControlMode.Velocity, velocityRadPerSec);
    }

    @Override
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
