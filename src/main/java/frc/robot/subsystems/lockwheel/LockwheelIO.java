package frc.robot.subsystems.lockwheel;

import org.littletonrobotics.junction.AutoLog;

public interface LockwheelIO {
    @AutoLog
    public static class LockwheelIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LockwheelIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void setVoltageUpMotor(double volts) {}

    public default void setVoltageDownMotor(double volts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec) {}

    public default void setVelocityUpMotor(double velocityRadPerSec) {}

    public default void setVelocityDownMotor(double velocityRadPerSec) {}

    /** Stop in open loop. */
    public default void stop() {}

    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {}
}
