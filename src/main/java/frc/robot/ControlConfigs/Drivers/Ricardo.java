package frc.robot.ControlConfigs.Drivers;

import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class Ricardo extends PlayerConfigs {
    public void getDriverConfig() {
        //Constants
        PlayerConfigs.turnSpeed = 0.65;
        PlayerConfigs.driveSpeed = 0.75;
        PlayerConfigs.fineTurnSpeed = 0.15;
        PlayerConfigs.fineDriveSpeed = 0.15;

        //Driving and rotation
        PlayerConfigs.xMovement = Robot.controller0.getLeftX();
        PlayerConfigs.yMovement = Robot.controller0.getLeftY();
        PlayerConfigs.turnMovement = Robot.controller0.getRightX();
        PlayerConfigs.fineControlToggle = Robot.controller0.getLeftBumper();
        PlayerConfigs.snapUp = Robot.controller0.getPOV() == 0;
        PlayerConfigs.snapRight = Robot.controller0.getPOV() == 90;
        PlayerConfigs.snapDown = Robot.controller0.getPOV() == 180;
        PlayerConfigs.snapLeft = Robot.controller0.getPOV() == 270;
        PlayerConfigs.align = Robot.controller0.getLeftTriggerAxis() > 0.2;

        //Scoring
        PlayerConfigs.fire = Robot.controller0.getRightBumper();
    } 

    public void getoperatorConfig() {
        //Intake
        PlayerConfigs.ground = Robot.controller1.getPOV() == 0;
        PlayerConfigs.amp = Robot.controller1.getPOV() == 90;
        PlayerConfigs.stow = Robot.controller1.getPOV() == 180;
        
        // Grabbing and rejecting objects
        PlayerConfigs.intake = Robot.controller1.getXButton();
        PlayerConfigs.reject = Robot.controller1.getBButton();

        //Intake Fine Control
        PlayerConfigs.fcEnable = Robot.controller1.getStartButton();
        PlayerConfigs.fcElbow = Robot.controller1.getLeftY();
        PlayerConfigs.fcWrist = Robot.controller1.getRightY();

        //Shooter Spin up
        PlayerConfigs.armScoringMechanism = Robot.controller1.getAButton();
        PlayerConfigs.shooterActive = Robot.controller1.getBackButton();
        
        //Climbers
        PlayerConfigs.splitClimberControl = Robot.controller1.getYButton();
        PlayerConfigs.climberLUp = Robot.controller1.getLeftBumper();
        PlayerConfigs.climberLDown = Robot.controller1.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.climberRUp = Robot.controller1.getRightBumper();
        PlayerConfigs.climberRDown = Robot.controller1.getRightTriggerAxis() > 0.2;
    }
}
