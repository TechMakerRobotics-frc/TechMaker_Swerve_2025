package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;

public class RetractIntakeCommand extends InstantCommand {

    public RetractIntakeCommand(Intake intake) {
        super(intake::retract, intake);
    }
}
