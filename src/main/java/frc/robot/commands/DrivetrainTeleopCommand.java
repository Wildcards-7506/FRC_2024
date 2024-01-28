package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.Robot;

public class DrivetrainTeleopCommand extends Command{

    double inputRot, yInputSpeed, xInputSpeed;

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
        // if(PlayerConfigs.snapZero){
        //     Robot.drivetrain.snap(0);
        // } else if(PlayerConfigs.snap90) {
        //     Robot.drivetrain.snap(90);
        // } else if(PlayerConfigs.snap180) {
        //     Robot.drivetrain.snap(180);
        // } else if(PlayerConfigs.snap270){
        //     Robot.drivetrain.snap(270);
        // } else if((PlayerConfigs.align)){
        //     Robot.drivetrain.align(Robot.limelight.getTX());
        // } else if (Math.abs(PlayerConfigs.xMovement) > 0.05 || Math.abs(PlayerConfigs.yMovement) > 0.05 || Math.abs(PlayerConfigs.turnMovement) > 0.05) {
        //     Robot.drivetrain.drive(yInputSpeed, xInputSpeed, inputRot, true, true);
        // } else {
        //     Robot.drivetrain.setX();
        // }
    }
}