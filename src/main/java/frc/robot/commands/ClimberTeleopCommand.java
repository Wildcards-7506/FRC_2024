package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberTeleopCommand extends Command{
    
    public ClimberTeleopCommand(){
        addRequirements(Robot.climbers);
    }
    
    @Override
    public void execute (){
        
    } 
}
