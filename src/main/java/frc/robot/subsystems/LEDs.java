package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    private AddressableLED ledString;
    private AddressableLEDBuffer ledBuffer;
    private int size;

    // Store what the last hue of the first pixel is
    private int m_rainbowFirstPixelHue;

    public LEDs(int PWMPort, int BufferSize){
        size = BufferSize;
        ledString = new AddressableLED(PWMPort);
        ledBuffer = new AddressableLEDBuffer(BufferSize);

        ledString.setLength(BufferSize);
        ledString.setData(ledBuffer);
        ledString.start();
    }

    public void update() {
      ledString.setData(ledBuffer);
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < this.size; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / this.size)) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        update();
    }

    public void solid(int hue, int sat, int val) {
        // For every pixel
        for (var i = 0; i < this.size; i++) {
          // Set the value
          ledBuffer.setHSV(i, hue, sat, val);
        }
        update();
    }
}