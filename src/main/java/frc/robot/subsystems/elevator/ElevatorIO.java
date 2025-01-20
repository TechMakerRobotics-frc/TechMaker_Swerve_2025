package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocityRadPerSec) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}

}