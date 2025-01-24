package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedConstants;

public class LedRainbow extends Command {

    private final Led leds;
    private int rainbowFirstPixelHue;

    public LedRainbow(Led leds) {
        this.leds = leds;
        rainbowFirstPixelHue = 0;
    }

  @Override
  public void initialize() {
    addRequirements(leds);
  }

    @Override
    public void execute() {
        for (var i = 0; i < LedConstants.LED_LENGHT; i++) {
            final int hue =
                    (rainbowFirstPixelHue + (i * 180 / LedConstants.LED_LENGHT)) % 180;
            leds.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
  }
}
