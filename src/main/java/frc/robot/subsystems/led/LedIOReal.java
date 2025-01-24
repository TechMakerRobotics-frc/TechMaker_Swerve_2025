package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOReal implements LedIO {
    private AddressableLED led = new AddressableLED(LedConstants.PWM_PORT);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LedConstants.LED_LENGHT);
    private int rainbowFirstPixelHue;
    private int currentPixel = 0; // Para efeitos em sequência

    public LedIOReal() {
        // Define o comprimento do LED
        led.setLength(LedConstants.LED_LENGHT);

        led.setData(ledBuffer);

        led.start();
        rainbowFirstPixelHue = 0;
        clear();
    }

    /** @param inputs */
    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.length = LedConstants.LED_LENGHT;  
        for (int i = 0; i < inputs.length; i++) {
            int[] color = getLEDColor(i);
            inputs.ledRed[i] = color[0];
            inputs.ledGreen[i] = color[1];
            inputs.ledBlue[i] = color[2];
        }  
    }

    /** Método para criar o efeito arco-íris */
    public void rainbow() {
        for (var i = 0; i < LedConstants.LED_LENGHT; i++) {
            final int hue = (rainbowFirstPixelHue + (i * 180 / LedConstants.LED_LENGHT)) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    @Override
    public void setHSV(int index, int h, int s, int v) {
        ledBuffer.setHSV(index, h, s, v);
    }

    /** Define todos os LEDs para uma cor RGB */
    @Override
    public void setAllColorRGB(int red, int green, int blue) {
        for (var i = 0; i < LedConstants.LED_LENGHT; i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        led.setData(ledBuffer);
    }

    /** Define todos os LEDs para uma cor HSV */
    @Override
    public void setAllColorHSV(int hue, int saturation, int value) {
        for (var i = 0; i < LedConstants.LED_LENGHT; i++) {
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
    public void gradientEffect(int redStart, int greenStart, int blueStart, int redEnd, int greenEnd, int blueEnd) {
        for (var i = 0; i < LedConstants.LED_LENGHT; i++) {
            double ratio = (double) i / (LedConstants.LED_LENGHT - 1);
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

    @Override
    public void clear() {
        setAllColorRGB(0,0, 0);
    }

    private int[] getLEDColor(int index) {
        int red = ledBuffer.getRed(index);
        int green = ledBuffer.getGreen(index);
        int blue = ledBuffer.getBlue(index);
        return new int[] { red, green, blue };
    }
}