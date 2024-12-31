package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class ExtendIntakeCommand extends InstantCommand {

    public ExtendIntakeCommand(Intake intake) {
        super(intake::extend, intake);
    }
}
