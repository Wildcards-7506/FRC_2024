package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ClimberTeleopCommand extends Command{

    private boolean prev_climberEngage = false;
    
    public ClimberTeleopCommand(){
        addRequirements(Robot.climbers);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.climberEngage != prev_climberEngage){
            prev_climberEngage = PlayerConfigs.climberEngage;
            if(PlayerConfigs.climberEngage){
                Robot.climbers.climberEngage = !Robot.climbers.climberEngage;
            }
        }

        //VERIFY DIRECTION
        if(!Robot.shooter.shootingMode && (Robot.intake.intakeState == 0 || Robot.intake.intakeState == 3)){
            if(Robot.climbers.climberEngage){
                Robot.climbers.setClimbers(ClimberConstants.kClimbPosition);
            } else{
                Robot.climbers.setClimbers(ClimberConstants.kRetractPosition);
            }
        }

        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Left Climber Position", Robot.climbers.getClimberLEncoder());
        SmartDashboard.putNumber("Right Climber Position", Robot.climbers.getClimberREncoder());
    } 
}
