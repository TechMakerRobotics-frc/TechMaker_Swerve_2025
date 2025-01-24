package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LedIOSim implements LedIO {
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
  private final AddressableLEDSim ledSim;
  private final int length;

  public LedIOSim() {
    led = new AddressableLED(LedConstants.PWM_PORT);
    ledSim = AddressableLEDSim.createForChannel(LedConstants.PWM_PORT);
    ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGHT);
    ledSim.setLength(LedConstants.LED_LENGHT);
    led.setLength(LedConstants.LED_LENGHT);
    led.setData(ledBuffer);
    led.start();

    length = LedConstants.LED_LENGHT;

    // Inicializar os LEDs desligados
    clear();
  }

  /**
   * @param inputs
   */
  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.length = length;
    for (int i = 0; i < inputs.length; i++) {
      int[] color = getSimulatedLEDColor(i);
      inputs.ledRed[i] = color[0];
      inputs.ledGreen[i] = color[1];
      inputs.ledBlue[i] = color[2];
    }
  }

  /**
   * Define a cor de todos os LEDs.
   *
   * @param red Valor do canal vermelho (0-255).
   * @param green Valor do canal verde (0-255).
   * @param blue Valor do canal azul (0-255).
   */
  @Override
  public void setAllColorRGB(int red, int green, int blue) {
    for (int i = 0; i < length; i++) {
      ledBuffer.setRGB(i, red, green, blue);
    }

    byte[] ledData = new byte[length * 4];
    for (int i = 0; i < length; i++) {
      int baseIndex = i * 4;
      ledData[baseIndex] = 0;
      ledData[baseIndex + 1] = (byte) red;
      ledData[baseIndex + 2] = (byte) green;
      ledData[baseIndex + 3] = (byte) blue;
    }

    ledSim.setData(ledData);
    led.setData(ledBuffer);
  }

  @Override
  public void setHSV(int index, int h, int s, int v) {
    ledBuffer.setHSV(index, h, s, v);
  }

  /** Define todos os LEDs para uma cor HSV */
  @Override
  public void setAllColorHSV(int hue, int saturation, int value) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, saturation, value);
    }

    byte[] ledData = new byte[length * 4];
    for (int i = 0; i < length; i++) {
      int baseIndex = i * 4;
      ledData[baseIndex] = 0;
      ledData[baseIndex + 1] = (byte) hue;
      ledData[baseIndex + 2] = (byte) saturation;
      ledData[baseIndex + 3] = (byte) value;
    }

    ledSim.setData(ledData);
  }

  /** Limpa todos os LEDs, desligando-os. */
  @Override
  public void clear() {
    setAllColorRGB(0, 0, 0);
  }

  private int[] getSimulatedLEDColor(int index) {
    byte[] ledData = ledSim.getData();
    int baseIndex = index * 4;
    int red = ledData[baseIndex + 1] & 0xFF;
    int green = ledData[baseIndex + 2] & 0xFF;
    int blue = ledData[baseIndex + 3] & 0xFF;
    return new int[] {red, green, blue};
  }
}
