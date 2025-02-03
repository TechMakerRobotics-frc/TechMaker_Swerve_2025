package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
  @AutoLog
  public static class LedIOInputs {
    public int length = LedConstants.LED_LENGHT;
    // Arrays para armazenar o R, G, B de cada LED
    public int[] ledRed;
    public int[] ledGreen;
    public int[] ledBlue;

    // Construtor para inicializar os arrays
    public LedIOInputs() {
      this.ledRed = new int[LedConstants.LED_LENGHT];
      this.ledGreen = new int[LedConstants.LED_LENGHT];
      this.ledBlue = new int[LedConstants.LED_LENGHT];
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LedIOInputs inputs) {}

  public default void setAllColorRGB(int red, int green, int blue) {}

  public default void setAllColorHSV(int hue, int saturation, int value) {}

  public default void setHSV(int index, int h, int s, int v) {}

  public default void clear() {}
}
