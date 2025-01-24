package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class LedRGB extends InstantCommand {

  public LedRGB(Leds leds, int r, int g, int b) {
    super(() -> leds.setAllColorRGB(r, g, b), leds);
  }
}
