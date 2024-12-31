package frc.robot.subsystems.lockwheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.util.subsystemUtils.LockwheelSim;

public class LockwheelIOSim implements LockwheelIO {
    // Novo construtor do LockwheelSim usando LinearSystemId.createFlywheelSystem
    private LockwheelSim sim = new LockwheelSim(
            LinearSystemId.createFlywheelSystem(
                    DCMotor.getNEO(1), // Modelo do motor
                    1.5, // Razão de transmissão
                    0.004 // Momento de inércia (kg·m²)
                    ),
            DCMotor.getNEO(1) // Gearbox (modelo do motor)
            );

    private PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(LockwheelIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = 0.0;
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        closedLoop = true;
        pid.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
}
