package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private AddressableLED led = new AddressableLED(9);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(240);
  private int rainbowFirstPixelHue;
  private int currentPixel = 0; // Para efeitos em sequência

  public Leds() {
    // Define o comprimento do LED
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);

    led.start();
    // Inicializa a variável para o arco-íris
    rainbowFirstPixelHue = 0;
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }

  /** Método para criar o efeito arco-íris */
  public void rainbow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }

  public void setHSV(int index, int h, int s, int v) {
    ledBuffer.setHSV(index, h, s, v);
  }

  /** Define todos os LEDs para uma cor RGB */
  public void setAllColorRGB(int red, int green, int blue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, red, green, blue);
    }
    led.setData(ledBuffer);
  }

  /** Define todos os LEDs para uma cor HSV */
  public void setAllColorHSV(int hue, int saturation, int value) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, hue, saturation, value);
    }
    led.setData(ledBuffer);
  }

  /** Efeito: Acender LEDs em sequência */
  public void chaseEffect(int red, int green, int blue) {
    // Apaga todos os LEDs primeiro
    setAllColorRGB(0, 0, 0);

    // Acende o próximo LED
    ledBuffer.setRGB(currentPixel, red, green, blue);

    // Incrementa para o próximo LED
    currentPixel = (currentPixel + 1) % ledBuffer.getLength();

    // Atualiza os LEDs
    led.setData(ledBuffer);
  }

  /** Efeito: Piscar todos os LEDs */
  public void blinkEffect(int red, int green, int blue, boolean isOn) {
    if (isOn) {
      setAllColorRGB(red, green, blue); // Liga com a cor especificada
    } else {
      setAllColorRGB(0, 0, 0); // Desliga
    }
  }

  /** Efeito: Gradiente de uma cor para outra */
  public void gradientEffect(
      int redStart, int greenStart, int blueStart, int redEnd, int greenEnd, int blueEnd) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      double ratio = (double) i / (ledBuffer.getLength() - 1);
      int red = (int) (redStart + ratio * (redEnd - redStart));
      int green = (int) (greenStart + ratio * (greenEnd - greenStart));
      int blue = (int) (blueStart + ratio * (blueEnd - blueStart));
      ledBuffer.setRGB(i, red, green, blue);
    }
    led.setData(ledBuffer);
  }

  public AddressableLEDBuffer getLedBuffer() {
    return ledBuffer;
  }
}
