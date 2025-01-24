package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class LedYellow extends InstantCommand {

  public LedYellow(Leds leds) {
    super(() -> leds.setAllColorRGB(255, 255, 0), leds);
  }
}
