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
        PlayerConfigs.fineControlToggle = Robot.controller0.getRightBumper();
        PlayerConfigs.snapUp = Robot.controller0.getPOV() == 0;
        PlayerConfigs.snapRight = Robot.controller0.getPOV() == 90;
        PlayerConfigs.snapDown = Robot.controller0.getPOV() == 180;
        PlayerConfigs.snapLeft = Robot.controller0.getPOV() == 270;
        PlayerConfigs.align = Robot.controller0.getLeftBumper();

        //Scoring and grabbing objects
        PlayerConfigs.fire = Robot.controller0.getRightTriggerAxis() > 0.2;
    } 

    public void getoperatorConfig() {
        //Intake
        PlayerConfigs.intake = Robot.controller1.getLeftTriggerAxis()>0.2;
        PlayerConfigs.trap = Robot.controller1.getXButton();
        PlayerConfigs.amp = Robot.controller1.getBButton();
        PlayerConfigs.stow = Robot.controller1.getLeftBumper();

        //Intake Fine Control
        PlayerConfigs.fcEnable = Robot.controller1.getStartButton();
        PlayerConfigs.fcElbow = Robot.controller1.getLeftY();
        PlayerConfigs.fcWrist = Robot.controller1.getRightY();

        //Shooter Spin up
        PlayerConfigs.armScoringMechanism = Robot.controller1.getRightTriggerAxis() > 0.2;
        PlayerConfigs.shooterActive = Robot.controller1.getRightBumper();
        
        //Climbers
        PlayerConfigs.climberUp = Robot.controller1.getPOV() == 180;
        PlayerConfigs.climberDown = Robot.controller1.getPOV() == 0;
    }
}
