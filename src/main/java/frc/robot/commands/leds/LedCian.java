package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedCian extends InstantCommand {

  public LedCian(Led leds) {
    super(() -> leds.setAllColorRGB(0, 255, 255), leds);
  }
}
