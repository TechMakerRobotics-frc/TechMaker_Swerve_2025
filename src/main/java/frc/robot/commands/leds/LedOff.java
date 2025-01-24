package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.leds.Leds;

public class LedOff extends InstantCommand {

  public LedOff(Leds leds) {
    super(() -> leds.setAllColorRGB(0, 0, 0), leds);
  }
}
