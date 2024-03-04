package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ClimberTeleopCommand extends Command{

    private boolean prev_SplitButton = false;
    
    public ClimberTeleopCommand(){
        addRequirements(Robot.climbers);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.splitClimberControl != prev_SplitButton){
            prev_SplitButton = PlayerConfigs.splitClimberControl;
            if(PlayerConfigs.splitClimberControl){
                Robot.climbers.splitControlMode = !Robot.climbers.splitControlMode;
            }
        }

        //VERIFY DIRECTION
        if(!Robot.shooter.shootingMode && (Robot.intake.intakeState == 0 || Robot.intake.intakeState == 3)){
            if(Robot.climbers.splitControlMode){
                if (PlayerConfigs.climberLDown) {
                    Robot.climbers.setLClimber(12);
                } else if (PlayerConfigs.climberLUp) {
                    Robot.climbers.setLClimber(-12);
                } else {
                    Robot.climbers.setLClimber(0);
                }
                if (PlayerConfigs.climberRDown) {
                    Robot.climbers.setRClimber(12);
                } else if (PlayerConfigs.climberRUp) {
                    Robot.climbers.setRClimber(-12);
                } else {
                    Robot.climbers.setRClimber(0);
                }
            } else {
                if (PlayerConfigs.climberLDown || PlayerConfigs.climberRDown) {
                    Robot.climbers.setLClimber(12);
                    Robot.climbers.setRClimber(12);
                } else if (PlayerConfigs.climberLUp || PlayerConfigs.climberRUp) {
                    Robot.climbers.setLClimber(-12);
                    Robot.climbers.setRClimber(-12);
                } else {
                    Robot.climbers.setLClimber(0);
                    Robot.climbers.setRClimber(0);
                }
            }
            
        }

        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Climber Position", Robot.climbers.getClimberEncoder());
    } 
}
