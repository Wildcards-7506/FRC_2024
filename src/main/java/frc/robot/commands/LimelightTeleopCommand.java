package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.LimelightConstants;

public class LimelightTeleopCommand extends Command{
    
    public LimelightTeleopCommand(){
        addRequirements(Robot.limelight);
    }
    
    @Override
    public void execute (){
        Robot.limelight.updateData();
        if(Robot.shooter.shootingMode){
            Robot.limelight.setLimelightPosition(LimelightConstants.kShooterPosition);
        } else {
            Robot.limelight.setLimelightPosition(LimelightConstants.kIntakePosition);
        }

        SmartDashboard.putNumber("Limelight Position", Robot.limelight.getPos());
        Robot.limelight.limelightLog();
    } 
}
