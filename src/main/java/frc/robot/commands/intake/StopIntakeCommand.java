package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class StopIntakeCommand extends InstantCommand {

    public StopIntakeCommand(Intake intake) {
        super(intake::stop, intake);
    }
}
