package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeTeleopCommand extends Command{
    
    public IntakeTeleopCommand(){
        addRequirements(Robot.intake);
    }
    
    @Override
    public void execute (){
        
    } 
}
