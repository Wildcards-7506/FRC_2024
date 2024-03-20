package frc.robot.ControlConfigs.Drivers;

import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class TestController extends PlayerConfigs {
    public void getDriverConfig() {
        //Constants
        PlayerConfigs.fullTurnSpeed = 1.0;
        PlayerConfigs.fullDriveSpeed = 1.0;
        PlayerConfigs.setupTurnSpeed = 0.5;
        PlayerConfigs.setupDriveSpeed = 0.5;
        PlayerConfigs.fineTurnSpeed = 0.15;
        PlayerConfigs.fineDriveSpeed = 0.15;

        //Driving and rotation
        PlayerConfigs.xMovement = Robot.controller0.getLeftX();
        PlayerConfigs.yMovement = Robot.controller0.getLeftY();
        PlayerConfigs.turnMovement = Robot.controller0.getRightX();
        PlayerConfigs.fineControlToggle = Robot.controller0.getLeftBumper();
        PlayerConfigs.setupControlToggle = Robot.controller0.getLeftTriggerAxis() > 0.2;
        PlayerConfigs.snapUp = Robot.controller0.getPOV() == 0;
        PlayerConfigs.snapRight = Robot.controller0.getPOV() == 90;
        PlayerConfigs.snapDown = Robot.controller0.getPOV() == 180;
        PlayerConfigs.snapLeft = Robot.controller0.getPOV() == 270;

        //Scoring
        PlayerConfigs.fire = Robot.controller0.getRightBumper();
    } 

    public void getOperatorConfig() {
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
        PlayerConfigs.climberEngage = Robot.controller1.getYButton();

    }
}