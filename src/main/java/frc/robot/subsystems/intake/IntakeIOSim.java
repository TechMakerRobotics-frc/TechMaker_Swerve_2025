package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.util.subsystemUtils.IntakeSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
    // Novo construtor do IntakeSim usando LinearSystemId.createFlywheelSystem
    private IntakeSim sim = new IntakeSim(
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

    private final IntakeSimulation intakeSimulation;

    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                // Specify the type of game pieces that the intake can collect
                "Coral",
                // Specify the drivetrain to which this intake is attached
                driveTrain,
                // Width of the intake
                Meters.of(0.7),
                // The extension length of the intake beyond the robot's frame (when activated)
                IntakeSimulation.IntakeSide.BACK,
                // The intake can hold up to 1 coral
                1);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
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

    @Override // Defined by IntakeIO
    public void extend() {
        intakeSimulation
                .startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game
        // pieces
    }

    @Override // Defined by IntakeIO
    public void retract() {
        intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
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
