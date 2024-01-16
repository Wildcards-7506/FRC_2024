package frc.robot.ControlConfigs.Drivers;

import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class TestController extends PlayerConfigs {
    public void getDriverConfig() {
        //Constants
        PlayerConfigs.turnSpeed = 0.5;
        PlayerConfigs.driveSpeed = 0.5;
        PlayerConfigs.fineTurnSpeed = 0.25;
        PlayerConfigs.fineDriveSpeed = 0.25;

        //Driving and rotation
        PlayerConfigs.xMovement = Robot.controller0.getLeftX();
        PlayerConfigs.yMovement = Robot.controller0.getLeftY();
        PlayerConfigs.turnMovement = Robot.controller0.getRightX();
        PlayerConfigs.fineControlToggle = Robot.controller0.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.xToggle = Robot.controller0.getAButton();
        PlayerConfigs.snapZero = false;
        PlayerConfigs.snap90 = false;
        PlayerConfigs.snap180 = false;
        PlayerConfigs.snap270 = false;
        PlayerConfigs.align = Robot.controller0.getLeftBumper();

        //Scoring and grabbing objects
        PlayerConfigs.shooterPrimed = Robot.controller0.getRightTriggerAxis() > 0.2;
        PlayerConfigs.shooterArmed = Robot.controller0.getRightBumper();
    } 

    public void getCoDriverConfig() {
        //Intake
        PlayerConfigs.intake = Robot.controller1.getRightBumper();
        PlayerConfigs.release = Robot.controller1.getRightTriggerAxis() > 0.2;
        PlayerConfigs.trap = Robot.controller1.getAButton();

        //Climbers
        PlayerConfigs.climberUp = Robot.controller1.getLeftBumper();
        PlayerConfigs.climberDown = Robot.controller1.getLeftTriggerAxis() > 0.2; 
    }
}