package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.Leds;

public class LedRainbow extends Command {

  private final Leds leds;
  private int rainbowFirstPixelHue;

  public LedRainbow(Leds leds) {
    this.leds = leds;
    rainbowFirstPixelHue = 0;
  }

  @Override
  public void initialize() {
    addRequirements(leds);
  }

  @Override
  public void execute() {
    for (var i = 0; i < leds.getLedBuffer().getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / leds.getLedBuffer().getLength())) % 180;
      leds.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }
}
