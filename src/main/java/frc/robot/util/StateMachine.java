package frc.robot.util;

import frc.robot.commands.flywheel.OutsideFlywheelCommand;
import frc.robot.commands.lockwheel.AlignBall;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lockwheel.Lockwheel;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {

    private final ScheduledExecutorService scheduler;

    public enum RobotState {
        WITHOUT_ELEMENT,
        CHARGING,
        WITH_NOT_ALIGNED_ELEMENT,
        READY_TO_SHOOT,
        SHOOTING,
    }

    private RobotState currentState;

    private final Lockwheel lockwheel;
    private final Flywheel flywheel;

    public StateMachine(Lockwheel lockwheel, Flywheel flywheel) {
        this.lockwheel = lockwheel;
        this.flywheel = flywheel;
        this.currentState = RobotState.WITHOUT_ELEMENT;

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::updateState, 0, 20, TimeUnit.MILLISECONDS);
    }

    public void updateState() {
        performAction();
        switch (currentState) {
            case WITHOUT_ELEMENT:
                if ((lockwheel.backSensorIsTrue() && !lockwheel.frontSensorIsTrue())
                        || (!lockwheel.backSensorIsTrue() && lockwheel.frontSensorIsTrue())) {
                    currentState = RobotState.WITH_NOT_ALIGNED_ELEMENT;
                }
                break;

            case WITH_NOT_ALIGNED_ELEMENT:
                if (lockwheel.backSensorIsTrue() && lockwheel.frontSensorIsTrue()) {
                    currentState = RobotState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                if (flywheel.getVelocityRPM() >= 800) {
                    currentState = RobotState.SHOOTING;
                }
                break;

            case SHOOTING:
                if (!lockwheel.backSensorIsTrue() && !lockwheel.frontSensorIsTrue()) {
                    currentState = RobotState.WITHOUT_ELEMENT;
                }
                break;

            default:
                throw new IllegalStateException("Estado desconhecido: " + currentState);
        }
    }

    @AutoLogOutput(key = "StateMachine/CurrentState")
    public RobotState getCurrentState() {
        return currentState;
    }

    public void performAction() {
        switch (currentState) {
            case WITHOUT_ELEMENT:
                break;

            case WITH_NOT_ALIGNED_ELEMENT:
                new AlignBall(lockwheel).schedule();
                break;

            case READY_TO_SHOOT:
                new OutsideFlywheelCommand(flywheel, 800).schedule();
                break;

            case SHOOTING:
                break;

            default:
                break;
        }
    }

    public void stop() {
        scheduler.shutdown();
    }
}
