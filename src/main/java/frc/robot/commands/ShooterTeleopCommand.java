package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterTeleopCommand extends Command{
    
    public ShooterTeleopCommand(){
        addRequirements(Robot.shooter);
    }
    
    @Override
    public void execute (){
        
    } 
}
