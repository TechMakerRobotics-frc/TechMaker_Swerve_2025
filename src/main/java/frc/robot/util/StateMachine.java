package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.lockwheel.Lockwheel;
import frc.robot.util.zones.ZoneManager;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine {

    private final ZoneManager blueZoneManager;
    private final ZoneManager redZoneManager;

    private final Lockwheel lockwheel;

    private final ScheduledExecutorService scheduler;

    // Map para associar estados e seus comandos
    private final Map<String, Command> stateCommands;
    private String currentState = "WithoutElement"; // Estado inicial
    private final String blueSpeakerZoneName = "BlueSpeakerZone";
    private final String redSpeakerZoneName = "RedSpeakerZone";

    public StateMachine(ZoneManager blueSpeakerzone, ZoneManager redSpeakerzone, Lockwheel lockwheel) {
        this.blueZoneManager = blueSpeakerzone;
        this.redZoneManager = redSpeakerzone;
        this.lockwheel = lockwheel;

        stateCommands = Map.of(
                "WithElement", new InstantCommand(() -> lockwheel.rotateForward(), lockwheel),
                "WithoutElement", new InstantCommand(() -> lockwheel.stop(), lockwheel),
                "InBlueScoringZone", new InstantCommand(() -> System.out.println("In Scoring Zone"), lockwheel));

        scheduler = Executors.newScheduledThreadPool(1);
        scheduler.scheduleAtFixedRate(this::periodicVerify, 0, 20, TimeUnit.MILLISECONDS);
    }

    private void periodicVerify() {
        String newState = detectState();

        if (!newState.equals(currentState)) {
            currentState = newState;
            executeStateCommand();
        }
    }

    @AutoLogOutput(key = "StateMachine/State")
    private String detectState() {
        if (lockwheel.backSensorIsTrue()) {
            return "WithElement";
        } else if (isInBlueScoringZone()) {
            return "InBlueScoringZone";
        } else if (isInRedScoringZone()) {
            return "InRedScoringZone";
        } else {
            return "Is not in a ScoringZone";
        }
    }

    @AutoLogOutput(key = "StateMachine/StateZoneScoring")
    private boolean isInBlueScoringZone() {
        return blueZoneManager.getCurrentZone().equalsIgnoreCase(blueSpeakerZoneName);
    }

    @AutoLogOutput(key = "StateMachine/StateZoneRedScoring")
    private boolean isInRedScoringZone() {
        return redZoneManager.getCurrentZone().equalsIgnoreCase(redSpeakerZoneName);
    }

    private void executeStateCommand() {
    }

    public void stop() {
        scheduler.shutdown();
    }

    public String getCurrentState() {
        return currentState;
    }
}
