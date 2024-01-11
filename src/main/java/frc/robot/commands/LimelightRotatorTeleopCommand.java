package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LimelightRotatorTeleopCommand extends Command{
    
    public LimelightRotatorTeleopCommand(){
        addRequirements(Robot.llrotator);
    }
    
    @Override
    public void execute (){
        
    } 
}
