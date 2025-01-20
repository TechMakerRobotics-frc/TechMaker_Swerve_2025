package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void runVoltsIntakeMotor(double volts) {
        io.setVoltageRollerMotor(volts);
    }
    
    public void runVoltsRollerMotor(double volts) {
        io.setVoltageRollerMotor(volts);
    }

    
    public void runVelocityIntakeMotor(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocityIntakeMotor(velocityRadPerSec);
        Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
    }
    
    public void runVelocityRollerMotor(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocityRollerMotor(velocityRadPerSec);
        Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
    }

    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    public void stopAll() {
        io.stopAll();
    }

    public void stopIntakeMotor() {
        io.stopIntakeMotor();
    }

    public void stopRollerMotor() {
        io.stopIntakeMotor();
    }
}
