package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LimelightTeleopCommand extends Command{
    
    public LimelightTeleopCommand(){
        addRequirements(Robot.limelight);
    }
    
    @Override
    public void execute (){
        Robot.limelight.updateData();
    } 
}
