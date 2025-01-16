package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedRGB extends InstantCommand {

    public LedRGB(Led leds, int r, int g, int b) {
        super(() -> leds.setAllColorRGB(r, g, b), leds);
    }
}
