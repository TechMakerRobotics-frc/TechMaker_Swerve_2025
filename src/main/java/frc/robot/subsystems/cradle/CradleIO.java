package frc.robot.subsystems.cradle;

import org.littletonrobotics.junction.AutoLog;

public interface CradleIO {
    @AutoLog
    public static class CradleIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(CradleIOInputs inputs) {}

    public default void setVoltageForAll(double volts) {}

    public default void setVoltageAlfaMotor(double volts) {}

    public default void setVoltageBetaMotor(double volts) {}

    public default void setVoltageGamaMotor(double volts) {}

    public default void setVelocityForAll(double velocityRadPerSec) {}

    public default void setVelocityAlfaMotor(double velocityRadPerSec) {}

    public default void setVelocityBetaMotor(double velocityRadPerSec) {}

    public default void setVelocityGamaMotor(double velocityRadPerSec) {}

    public default void stopAll() {}

    public default void stopAlfa() {}

    public default void stopBeta() {}

    public default void stopGama() {}

    public default void configurePID(double kP, double kI, double kD) {}

}