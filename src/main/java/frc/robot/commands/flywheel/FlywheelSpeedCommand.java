package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class FlywheelSpeedCommand extends Command {

    private final Flywheel flywheel;
    private double ControllerInput;

    public FlywheelSpeedCommand(Flywheel flywheel, double ControllerInput) {
        this.flywheel = flywheel;
        this.ControllerInput = ControllerInput;
    }

    @Override
    public void execute() {
        flywheel.runVelocity(ControllerInput * 1500);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
