package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.leds.*;
import frc.robot.subsystems.led.Led;

public class RegisNamedCommands {

  private final Led leds;

  /** Register commands in the pathplanner. */
  public RegisNamedCommands(Led leds) {
    this.leds = leds;
    RegisterLeds();
  }

  private void RegisterLeds() {
    NamedCommands.registerCommand("Led Red", new LedRed(leds));
    NamedCommands.registerCommand("Led Green", new LedGreen(leds));
    NamedCommands.registerCommand("Led Blue", new LedBlue(leds));
    NamedCommands.registerCommand("Led Yellow", new LedYellow(leds));
    NamedCommands.registerCommand("Led Cian", new LedCian(leds));
    NamedCommands.registerCommand("Led White", new LedWhite(leds));
    NamedCommands.registerCommand("Led Rainbow", new LedRainbow(leds));
    NamedCommands.registerCommand("Led Off", new LedOff(leds));
  }
}
