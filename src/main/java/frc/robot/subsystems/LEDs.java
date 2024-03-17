package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase{
    private AddressableLED ledString;
    private AddressableLEDBuffer ledBuffer;
    private int m_rainbowFirstPixelHue;
    public int teamRainbow;
    public int alignOOB;
    public int offState;
    public int shooterLo;
    public boolean triggered;
    public Timer lightTimer = new Timer();

    public LEDs(){
        ledString = new AddressableLED(LEDConstants.pwmPort);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.bufferSize);

        ledString.setLength(LEDConstants.bufferSize);
        ledString.setData(ledBuffer);
        ledString.start();
    }

    public void update() {
      ledString.setData(ledBuffer);
    }

    public void rainbow(int colorScheme) {
      if(colorScheme == 1){
        //150 through 15
        // For every pixel
        for (var i = 0; i < LEDConstants.bufferSize; i++) {
          final var hue = 150+(m_rainbowFirstPixelHue + (i * 45 / LEDConstants.bufferSize)) % 45;
          // Set the value
          ledBuffer.setHSV(i, hue, LEDConstants.SATURATED, LEDConstants.FULL);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 45 / LEDConstants.bufferSize;
        // Check bounds
        m_rainbowFirstPixelHue %= 45;
      } else if(colorScheme == 2){
        //90 through 135
        // For every pixel
        for (var i = 0; i < LEDConstants.bufferSize; i++) {
          final var hue = 90+(m_rainbowFirstPixelHue + (i * 45 / LEDConstants.bufferSize)) % 45;
          // Set the value
          ledBuffer.setHSV(i, hue, LEDConstants.SATURATED, LEDConstants.FULL);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 45 / LEDConstants.bufferSize;
        // Check bounds
        m_rainbowFirstPixelHue %= 45;
      } else {
        // For every pixel
        for (var i = 0; i < LEDConstants.bufferSize; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / LEDConstants.bufferSize)) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue, LEDConstants.SATURATED, LEDConstants.FULL);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
        update();
    }

    public void solid(int hue, int sat, int val) {
        for (var i = 0; i < LEDConstants.bufferSize; i++) {
          ledBuffer.setHSV(i, hue, sat, val);
        }
        update();
    }

    public void section(int start, int finish, int hue, int sat, int val) {
        for (var i = start; i <= finish; i++) {
          ledBuffer.setHSV(i, hue, sat, val);
        }
        update();
    }
}