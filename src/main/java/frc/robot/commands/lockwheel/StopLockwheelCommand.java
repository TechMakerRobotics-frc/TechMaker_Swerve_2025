package frc.robot.commands.lockwheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.lockwheel.Lockwheel;

public class StopLockwheelCommand extends InstantCommand {

    public StopLockwheelCommand(Lockwheel lockwheel) {
        super(lockwheel::stop, lockwheel);
    }
}
