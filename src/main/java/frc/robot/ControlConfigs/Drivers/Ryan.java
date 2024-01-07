package frc.robot.ControlConfigs.Drivers;

import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class Ryan extends PlayerConfigs {
    public void getDriverConfig() {
        //Constants
        PlayerConfigs.turnSpeed = 0.3;
        PlayerConfigs.driveSpeed = 0.5;
        PlayerConfigs.fineTurnSpeed = 0.175;
        PlayerConfigs.fineDriveSpeed = 0.2;

        //Driving and rotation
        PlayerConfigs.xMovement = Robot.controller0.getLeftX();
        PlayerConfigs.yMovement = Robot.controller0.getLeftY();
        PlayerConfigs.turnMovement = Robot.controller0.getRightX();
        PlayerConfigs.fineControlToggle = Robot.controller0.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.xToggle = Robot.controller0.getRightTriggerAxis() > 0.2;
        PlayerConfigs.snapZero = Robot.controller0.getAButton();
        PlayerConfigs.snap90 = Robot.controller0.getBButton();
        PlayerConfigs.snap180 = Robot.controller0.getYButton();
        PlayerConfigs.snap270 = Robot.controller0.getXButton();
        PlayerConfigs.align = Robot.controller0.getLeftBumper();        
        PlayerConfigs.brake = Robot.controller0.getRightBumper();
    } 

    public void getCoDriverConfig() {
        //Scoring and grabbing objects
        PlayerConfigs.armControl = Robot.controller0.getPOV();
        PlayerConfigs.armFineControl = Robot.controller1.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.armPos = Robot.controller1.getLeftY();

        //Intake
        PlayerConfigs.intake = Robot.controller0.getRightTriggerAxis() > 0.2;
        PlayerConfigs.release = Robot.controller0.getRightBumper();

        //Limelight Switch
        PlayerConfigs.switchPipeline = Robot.controller1.getStartButton();
    }
}