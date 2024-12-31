package frc.robot.commands.lockwheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lockwheel.Lockwheel;

public class AlignBall extends Command {

    private final Lockwheel lockwheel;
    private boolean isFinished;

    public AlignBall(Lockwheel lockwheel) {
        this.lockwheel = lockwheel;

        addRequirements(lockwheel);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        boolean frontSensor = lockwheel.frontSensorIsTrue();
        boolean backSensor = lockwheel.backSensorIsTrue();

        if (frontSensor && backSensor) {
            isFinished = true;
            lockwheel.stop();
        } else if (frontSensor) {
            lockwheel.rotateForward();
        } else if (backSensor) {
            lockwheel.rotateBackward();
        } else {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        lockwheel.stop();
    }
}
