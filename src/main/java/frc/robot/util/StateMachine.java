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
        SEM_ELEMENTO,
        CARREGANDO,
        COM_ELEMENTO_NÃO_ALINHADO,
        PRONTO_PARA_LANCAR,
        LANCANDO,
    }

    private RobotState currentState;

    private final Lockwheel lockwheel;
    private final Flywheel flywheel;

    public StateMachine(Lockwheel lockwheel, Flywheel flywheel) {
        this.lockwheel = lockwheel;
        this.flywheel = flywheel;
        this.currentState = RobotState.SEM_ELEMENTO;

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::updateState, 0, 20, TimeUnit.MILLISECONDS);
    }

    public void updateState() {
        performAction();
        switch (currentState) {
            case SEM_ELEMENTO:
                if ((lockwheel.backSensorIsTrue() && !lockwheel.frontSensorIsTrue())
                        || (!lockwheel.backSensorIsTrue() && lockwheel.frontSensorIsTrue())) {
                    currentState = RobotState.COM_ELEMENTO_NÃO_ALINHADO;
                }
                break;

            case COM_ELEMENTO_NÃO_ALINHADO:
                if (lockwheel.backSensorIsTrue() && lockwheel.frontSensorIsTrue()) {
                    currentState = RobotState.PRONTO_PARA_LANCAR;
                }
                break;

            case PRONTO_PARA_LANCAR:
                if (flywheel.getVelocityRPM() >= 800) {
                    currentState = RobotState.LANCANDO;
                }
                break;

            case LANCANDO:
                if (!lockwheel.backSensorIsTrue() && !lockwheel.frontSensorIsTrue()) {
                    currentState = RobotState.SEM_ELEMENTO;
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
            case SEM_ELEMENTO:
                break;

            case COM_ELEMENTO_NÃO_ALINHADO:
                new AlignBall(lockwheel).schedule();
                break;

            case PRONTO_PARA_LANCAR:
                new OutsideFlywheelCommand(flywheel, 800).schedule();
                break;

            case LANCANDO:
                break;

            default:
                break;
        }
    }

    public void stop() {
        scheduler.shutdown();
    }
}
