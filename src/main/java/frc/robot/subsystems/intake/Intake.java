package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final DoubleSolenoid solenoid;

    public Intake(IntakeIO io) {
        this.io = io;
        solenoid = new DoubleSolenoid(
                IntakeConstants.SOLENOID_MODULE_CAN_ID,
                IntakeConstants.SOLENOID_MODULE_TYPE,
                IntakeConstants.SOLENOID_FORWARD_CHANNEL,
                IntakeConstants.SOLENOID_REVERSE_CHANNEL);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    public void runVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocity(velocityRadPerSec);
        Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
    }

    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    /** Stops the Intake. */
    public void stop() {
        io.stop();
    }

    /** Ativa o solenoide para o estado forward. */
    public void extend() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /** Ativa o solenoide para o estado reverse. */
    public void retract() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /** Desativa o solenoide. */
    public void off() {
        solenoid.set(DoubleSolenoid.Value.kOff);
    }
}
