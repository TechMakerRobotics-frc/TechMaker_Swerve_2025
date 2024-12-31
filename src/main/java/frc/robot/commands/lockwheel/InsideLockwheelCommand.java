package frc.robot.commands.lockwheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lockwheel.Lockwheel;

public class InsideLockwheelCommand extends Command {

    private final Lockwheel lockwheel;
    private final double velocity;

    public InsideLockwheelCommand(Lockwheel lockwheel, double velocity) {
        this.lockwheel = lockwheel;
        this.velocity = velocity;

        addRequirements(lockwheel);
    }

    @Override
    public void initialize() {
        lockwheel.runVelocity(-velocity);
    }

    @Override
    public void end(boolean interrupted) {
        lockwheel.stop();
    }
}
