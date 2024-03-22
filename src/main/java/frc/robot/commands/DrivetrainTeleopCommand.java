package frc.robot.commands;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.Robot;

public class DrivetrainTeleopCommand extends Command{

    double inputRot, yInputSpeed, xInputSpeed;
    String payload;
    private static final DecimalFormat df = new DecimalFormat("0.00");

    public DrivetrainTeleopCommand() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute(){
        //Reset Gyro
        if(PlayerConfigs.zeroGyro){
            Robot.drivetrain.zeroHeading();
        }

        //Joystick Inputs
        xInputSpeed = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineDriveSpeed * PlayerConfigs.xMovement :
            PlayerConfigs.setupControlToggle ?
            PlayerConfigs.setupDriveSpeed * PlayerConfigs.xMovement :
            PlayerConfigs.fullDriveSpeed * PlayerConfigs.xMovement;
        yInputSpeed = PlayerConfigs.fineControlToggle ? 
            -PlayerConfigs.fineDriveSpeed * PlayerConfigs.yMovement : 
            PlayerConfigs.setupControlToggle ?
            -PlayerConfigs.setupDriveSpeed * PlayerConfigs.yMovement :
            -PlayerConfigs.fullDriveSpeed * PlayerConfigs.yMovement;
        inputRot = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineTurnSpeed * PlayerConfigs.turnMovement : 
            PlayerConfigs.setupControlToggle ?
            PlayerConfigs.setupTurnSpeed * PlayerConfigs.turnMovement :
            PlayerConfigs.fullTurnSpeed * PlayerConfigs.turnMovement;

        //Snap or align if needed, set drive if joystick inputs available, otherwise X
        if(PlayerConfigs.snapUp){
            double angle = Robot.teamColor.get() == Alliance.Red ? 180 : 0;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapRight) {
            double angle = Robot.teamColor.get() == Alliance.Red ? 90 : -90;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapDown) {
            double angle = Robot.teamColor.get() == Alliance.Red ? 0 : 90;
            Robot.drivetrain.snap(angle);
        } else if(PlayerConfigs.snapLeft){
            double angle = Robot.teamColor.get() == Alliance.Red ? -90 : 90;
            Robot.drivetrain.snap(angle);
        } else if (Math.abs(PlayerConfigs.xMovement) > 0.05 || Math.abs(PlayerConfigs.yMovement) > 0.05 || Math.abs(PlayerConfigs.turnMovement) > 0.05) {
            Robot.drivetrain.drive(yInputSpeed, xInputSpeed, inputRot, true, true);
        } else {
            Robot.drivetrain.setX();
        }

        //Report Joysticks to Logger
        payload = df.format(yInputSpeed)
        + " " + df.format(xInputSpeed)
        + " " + df.format(inputRot)
        + " (Y,X,R)";
        
        Robot.drivetrain.driveLog(payload);
    }
}