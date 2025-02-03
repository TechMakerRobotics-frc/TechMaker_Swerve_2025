package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.Led;

public class LedYellow extends InstantCommand {

  public LedYellow(Led leds) {
    super(() -> leds.setAllColorRGB(255, 255, 0), leds);
  }
}
