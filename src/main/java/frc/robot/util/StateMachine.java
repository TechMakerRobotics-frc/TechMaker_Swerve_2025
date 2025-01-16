package frc.robot.util;

import frc.robot.commands.flywheel.OutsideFlywheelCommand;
import frc.robot.commands.lockwheel.AlignBall;
import frc.robot.subsystems.drive.Drive;
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
        READY_TO_ALIGN
    }

    private RobotState currentState;

    private final Drive drive;
    private final Lockwheel lockwheel;
    private final Flywheel flywheel;

    public StateMachine(Drive drive, Lockwheel lockwheel, Flywheel flywheel) {
        this.drive = drive;
        this.lockwheel = lockwheel;
        this.flywheel = flywheel;
        this.currentState = RobotState.WITHOUT_ELEMENT;

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::updateState, 0, 20, TimeUnit.MILLISECONDS);
    }

    public void updateState() {
        performAction();
        switch (currentState) {
            case READY_TO_ALIGN:
                /*if (!alignCommandScheduled) {
                    new AlignTo(drive, 7, 10).schedule();
                    alignCommandScheduled = true;
                }*/
                if (drive.getCurrentZone().equals("Is not in a zone")) {
                    currentState = RobotState.WITHOUT_ELEMENT;
                }
                break;

            case WITHOUT_ELEMENT:
                /*if ((lockwheel.backSensorIsTrue() && !lockwheel.frontSensorIsTrue())
                        || (!lockwheel.backSensorIsTrue() && lockwheel.frontSensorIsTrue())) {
                    currentState = RobotState.WITH_NOT_ALIGNED_ELEMENT;
                }*/
                if (drive.getCurrentZone().equals("BlueSpeakerZone")) {
                    currentState = RobotState.READY_TO_ALIGN;
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
            case READY_TO_ALIGN:
                break;
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

    public boolean isReadyToAlign() {
        switch (currentState) {
            case READY_TO_ALIGN:
                return true;
        }
        return false;
    }
}
