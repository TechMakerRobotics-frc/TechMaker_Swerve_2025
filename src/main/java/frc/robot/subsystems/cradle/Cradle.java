package frc.robot.subsystems.cradle;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Cradle extends SubsystemBase {

    private final CradleIO io;
    private final CradleIOInputsAutoLogged inputs = new CradleIOInputsAutoLogged();

    public Cradle(CradleIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Cradle", inputs);
    }

    public void runVolts(double volts) {
        io.setVoltageForAll(volts);
    }

    public void runVelocity(double velocityRPM) {
        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        io.setVelocityForAll(velocityRadPerSec);
        Logger.recordOutput("Cradle/SetpointRPM", velocityRPM);
    }

    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    /** Stops the Cradle. */
    public void stop() {
        io.stopAll();
    }
}
