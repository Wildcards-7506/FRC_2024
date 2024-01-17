package frc.robot.ControlConfigs.Drivers;

import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class Jayden extends PlayerConfigs {
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
        PlayerConfigs.fineControlToggle = false;
        PlayerConfigs.xToggle = false;
        PlayerConfigs.snapZero = false;
        PlayerConfigs.snap90 = false;
        PlayerConfigs.snap180 = false;
        PlayerConfigs.snap270 = false;
        PlayerConfigs.align = false;

        //Scoring and grabbing objects
        PlayerConfigs.shooterActive = false;
        PlayerConfigs.fire = false;
    } 

    public void getCoDriverConfig() {
        //Intake
        PlayerConfigs.intake = false;
        PlayerConfigs.trap = false;
        PlayerConfigs.amp = false;
        PlayerConfigs.stow = false;

        //Shooter Spin up
        PlayerConfigs.shooterArmed = false;
        
        //Climbers
        PlayerConfigs.climberUp = Robot.controller1.getLeftTriggerAxis () > 0.2;
        PlayerConfigs.climberDown = Robot.controller1.getRightTriggerAxis() > 0.2; 
    }
}
