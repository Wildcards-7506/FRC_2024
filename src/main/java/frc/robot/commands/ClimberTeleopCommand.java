package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ClimberTeleopCommand extends Command{
    
    public ClimberTeleopCommand(){
        addRequirements(Robot.climbers);
    }
    
    @Override
    public void execute (){
        //VERIFY DIRECTION
        if(!Robot.shooter.shootingMode && Robot.intake.getElbowEncoder() < 45){
            if(Robot.climbers.climberEngage){
                Robot.climbers.setClimbers(ClimberConstants.kEngagePosition);
            } else{
                Robot.climbers.setClimbers(ClimberConstants.kPrimePosition);
            }
        } else {
            Robot.climbers.setClimbers(ClimberConstants.kIdlePosition);
        }

        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Left Climber Position", Robot.climbers.getClimberLEncoder());
        SmartDashboard.putNumber("Right Climber Position", Robot.climbers.getClimberREncoder());
    } 
}
