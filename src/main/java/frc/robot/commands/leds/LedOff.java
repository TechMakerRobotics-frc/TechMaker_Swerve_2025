package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedOff extends InstantCommand {

    public LedOff(Led leds) {
        super(() -> leds.clear(), leds);
    }
}
