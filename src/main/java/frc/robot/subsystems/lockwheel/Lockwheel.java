package frc.robot.subsystems.lockwheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Lockwheel extends SubsystemBase {
    private final LockwheelIO io;
    private final LockwheelIOInputsAutoLogged inputs = new LockwheelIOInputsAutoLogged();

    private final DigitalInput frontSensor = new DigitalInput(LockwheelConstants.FRONT_SENSOR_CHANNEL);
    private final DigitalInput backSensor = new DigitalInput(LockwheelConstants.BACK_SENSOR_CHANNEL);

    public Lockwheel(LockwheelIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lockwheel", inputs);
    }

    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    public void runVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocity(velocityRadPerSec);
        Logger.recordOutput("Lockwheel/SetpointRPM", velocityRPM);
    }

    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    public void rotateForward() {
        runVelocity(LockwheelConstants.VELOCITY_ROTATE_FORWARD);
    }

    public void rotateBackward() {
        runVelocity(LockwheelConstants.VELOCITY_ROTATE_BACKWARD);
    }

    /** Stops the Lockwheel. */
    public void stop() {
        io.stop();
    }
    /**
     * Returns true if the sensor has an object.
     *
     * @return get sensor
     */
    @AutoLogOutput
    public boolean frontSensorIsTrue() {
        return !frontSensor.get();
    }

    /**
     * Returns true if the sensor has an object.
     *
     * @return get sensor
     */
    @AutoLogOutput
    public boolean backSensorIsTrue() {
        return !backSensor.get();
    }
}
