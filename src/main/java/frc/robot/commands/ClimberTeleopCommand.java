package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ClimberTeleopCommand extends Command{

    private boolean prev_SplitButton = false;
    private double lClimberVoltage = 0;
    private double rClimberVoltage = 0;
    
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
                lClimberVoltage = PlayerConfigs.climberLDown ? 12 : PlayerConfigs.climberLUp ? -12 : 0;
                rClimberVoltage = PlayerConfigs.climberRDown ? 12 : PlayerConfigs.climberRUp ? -12 : 0;
            } else {
                lClimberVoltage = PlayerConfigs.climberLDown || PlayerConfigs.climberRDown ? 12 : PlayerConfigs.climberLUp || PlayerConfigs.climberRUp ? -12 : 0;
                rClimberVoltage = lClimberVoltage;
            }
            Robot.climbers.setClimbers(lClimberVoltage, rClimberVoltage);
        }

        Robot.climbers.climberLog();
        SmartDashboard.putNumber("Climber Position", Robot.climbers.getClimberEncoder());
    } 
}
