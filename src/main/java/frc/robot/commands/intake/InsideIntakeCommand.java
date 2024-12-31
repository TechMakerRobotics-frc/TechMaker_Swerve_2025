package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class InsideIntakeCommand extends Command {

    private final Intake intake;
    private final double velocity;

    public InsideIntakeCommand(Intake intake, double velocity) {
        this.intake = intake;
        this.velocity = velocity;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.runVelocity(-velocity);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
