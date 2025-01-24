package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedBlue extends InstantCommand {

  public LedBlue(Led leds) {
    super(() -> leds.setAllColorRGB(0, 0, 255), leds);
  }
}
