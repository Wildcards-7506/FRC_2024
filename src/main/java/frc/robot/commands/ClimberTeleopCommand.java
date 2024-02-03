package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ClimberTeleopCommand extends Command{
    
    public ClimberTeleopCommand(){
        addRequirements(Robot.climbers);
    }
    
    @Override
    public void execute (){
        //VERIFY DIRECTION
        if(!Robot.shooter.shootingMode && (Robot.intake.intakeState == 0 || Robot.intake.intakeState == 3)){
            if (PlayerConfigs.climberDown) {
                Robot.climbers.setClimbers(-12);
            } else if (PlayerConfigs.climberUp) {
                Robot.climbers.setClimbers(12);
            } else {
                Robot.climbers.setClimbers(0);
            }
        }

        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Climber Position", Robot.climbers.getClimberEncoder());
    } 
}
