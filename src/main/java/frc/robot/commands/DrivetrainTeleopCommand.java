package frc.robot.commands;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.Robot;
import frc.robot.util.Logger;

public class DrivetrainTeleopCommand extends Command{

    double inputRot, yInputSpeed, xInputSpeed;
    String payload;
    private static final DecimalFormat df = new DecimalFormat("0.00");

    public DrivetrainTeleopCommand() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute(){
        xInputSpeed = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineDriveSpeed * PlayerConfigs.xMovement :
            PlayerConfigs.driveSpeed * PlayerConfigs.xMovement;
        yInputSpeed = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineDriveSpeed * PlayerConfigs.yMovement : 
            PlayerConfigs.driveSpeed * PlayerConfigs.yMovement;
        inputRot = PlayerConfigs.fineControlToggle ? 
            PlayerConfigs.fineTurnSpeed * PlayerConfigs.turnMovement : 
            PlayerConfigs.turnSpeed * PlayerConfigs.turnMovement;

        //Snap if needed, otherwise set drive motors
        if(PlayerConfigs.snapZero){
            Robot.drivetrain.snap(0);
        } else if(PlayerConfigs.snap90) {
            Robot.drivetrain.snap(90);
        } else if(PlayerConfigs.snap180) {
            Robot.drivetrain.snap(180);
        } else if(PlayerConfigs.snap270){
            Robot.drivetrain.snap(270);
        } else if((PlayerConfigs.align)){
            Robot.drivetrain.align(Robot.limelight.getTX());
        } else if (PlayerConfigs.xToggle) {
            Robot.drivetrain.setX();
        } else {
            Robot.drivetrain.drive(yInputSpeed, xInputSpeed, inputRot, true, true);
        }   
        payload = df.format(yInputSpeed)
         + " " + df.format(xInputSpeed)
         + " " + df.format(inputRot)
         + " (Y,X,R)";
        
         Logger.info("DRIVE", payload);
        Robot.drivetrain.m_frontLeft.errorCheck("FLDRV", "FLTRN");
        Robot.drivetrain.m_rearLeft.errorCheck("RLDRV", "RLTRN");
        Robot.drivetrain.m_frontRight.errorCheck("FRDRV", "FRTRN");
        Robot.drivetrain.m_rearRight.errorCheck("RRDRV", "RRTRN");
    }
}