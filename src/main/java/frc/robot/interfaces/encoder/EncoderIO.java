package frc.robot.interfaces.encoder;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    @AutoLog
    public static class EncoderIOInputs {
        public double absolutePosition = 0.0;        
        public double relativePosition = 0.0;        
        public double velocity = 0.0;
        public double pulseCount = 0.0;              
        public double rateOfChange = 0.0;             
        public double distanceTraveled = 0.0;    
        public double temperature = 0.0;
        public boolean isOverloaded = false;
        public boolean isFault = false;        
        public boolean isStopped = true;  
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(EncoderIOInputs inputs) {}

    public default EncoderIOInputs getEncoderIOInputs() {
        EncoderIOInputs inputs = new EncoderIOInputs();
        updateInputs(inputs);
        return inputs;
    }

    public default void reset() {}
}
