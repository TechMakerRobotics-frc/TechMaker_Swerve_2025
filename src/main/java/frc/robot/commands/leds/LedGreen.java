package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedGreen extends InstantCommand {

  public LedGreen(Led leds) {
    super(() -> leds.setAllColorRGB(0, 255, 0), leds);
  }
}
