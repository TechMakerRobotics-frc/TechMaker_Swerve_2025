package frc.robot.interfaces.motor;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
    @AutoLog
    public static class MotorIOInputs {
        public double positionRot = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(MotorIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec) {}

    /** Run closed loop position */
    public default void setPosition(double positionRad) {}

    /** Stop in open loop. */
    public default void stop() {}

    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {}

     /** Set velocity PIDF constants. */
     public default void configurePIDF(double kP, double kI, double kD, double kF) {}

    /** Reset encoder position. */
    public default void resetPosition() {}
    
    /** Sets encoder offset. */
    public default void setOffset(double offset) {}

    public default MotorIOInputs getMotorIOInputs() {
        
        MotorIOInputs inputs = new MotorIOInputs();
        updateInputs(inputs);
        return inputs;
    }
}
