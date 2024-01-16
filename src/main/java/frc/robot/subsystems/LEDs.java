package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Robot;

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
          ledBuffer.setHSV(i, hue, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
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

    public void section(int start, int finish, int hue, int sat, int val) {
        // For every pixel
        for (var i = start; i <= finish; i++) {
          // Set the value
          ledBuffer.setHSV(i, hue, sat, val);
        }
        update();
    }

    public void checkAlign(double alignment, boolean aligning) {
        if(aligning){
            if(alignment < 2){
                Robot.ledSystem.section(0, 9, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.section(0, 9, LEDConstants.ORANGE, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            }
        } else {
            Robot.ledSystem.section(0, 9, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        }
    }

    public void checkSpinup(double speed) {
        if(speed > 2800){
            Robot.ledSystem.section(10, 19, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.shooter.getSpeed() > 150){
            Robot.ledSystem.section(10, 19, LEDConstants.RED, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else{
            Robot.ledSystem.section(10, 19, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        }
    }

    public void checkIntake(double current, boolean intaking) {
        if(intaking){
            if(current < 10){
                Robot.ledSystem.section(20, 29, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.section(20, 29, LEDConstants.MAGENTA, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            }
        } else {
            Robot.ledSystem.section(20, 29, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        }
    }

    public void checkClimberHeight(double height) {
        if(height > 2){
            if(Robot.climbers.getClimberEncoder() > 22){
                Robot.ledSystem.solid(LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.rainbow();
            }
        }
    }
}