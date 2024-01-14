package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.ledSystem);
    }
    
    @Override
    public void execute (){
        Robot.ledSystem.checkAlign(Math.abs(Robot.limelight.getTX()), PlayerConfigs.align);
        Robot.ledSystem.checkSpinup(Robot.shooter.getSpeed());
        Robot.ledSystem.checkIntake(Robot.intake.getIntakeCurrent(), PlayerConfigs.intake);
        Robot.ledSystem.checkClimberHeight(Robot.climbers.getClimberEncoder());
    } 
}
