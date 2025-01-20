package frc.robot.interfaces.encoder;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class EncoderIOThroughBore implements EncoderIO {

    private final Encoder encoder;  
    private final DutyCycleEncoder dutyCycleEncoder;  
    private boolean isMotorEncoder;

    public EncoderIOThroughBore(int channelA, int channelB, double distancePerPulse) {
        encoder = new Encoder(channelA, channelB, false, EncodingType.k1X);
        encoder.setDistancePerPulse(distancePerPulse);
        dutyCycleEncoder = null;
        isMotorEncoder = false;
    }

    public EncoderIOThroughBore(SparkMax motor) {
        dutyCycleEncoder = new DutyCycleEncoder(motor.);
        encoder = null;
        isMotorEncoder = true;
    }

    /** @param inputs */
    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        if (isMotorEncoder) {
            inputs.absolutePosition = dutyCycleEncoder.get();  
        } else {
            inputs.absolutePosition = encoder.get();            
            inputs.velocity = encoder.getRate();               
            inputs.pulseCount = encoder.getRaw();            
            inputs.rateOfChange = encoder.getRate();    
            inputs.distanceTraveled = encoder.getDistance();
            inputs.isStopped = encoder.getStopped();
        }
    }

    @Override
    public void reset() {
        if (isMotorEncoder) return;
        else {
            encoder.reset();
        }
    }
}
