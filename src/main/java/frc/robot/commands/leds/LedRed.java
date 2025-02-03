package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedRed extends InstantCommand {

  public LedRed(Led leds) {
    super(() -> leds.setAllColorRGB(255, 0, 0), leds);
  }
}
