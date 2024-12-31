package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

public class InsideFlywheelCommand extends Command {

    private final Flywheel flywheel;
    private final double velocity;

    public InsideFlywheelCommand(Flywheel flywheel, double velocity) {
        this.flywheel = flywheel;
        this.velocity = velocity;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.runVelocity(-velocity);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}
