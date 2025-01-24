package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class LedWhite extends InstantCommand {

  public LedWhite(Leds leds) {
    super(() -> leds.setAllColorRGB(255, 255, 255), leds);
  }
}
