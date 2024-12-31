package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.flywheel.Flywheel;

public class StopFlywheelCommand extends InstantCommand {

    public StopFlywheelCommand(Flywheel flywheel) {
        super(flywheel::stop, flywheel);
    }
}
