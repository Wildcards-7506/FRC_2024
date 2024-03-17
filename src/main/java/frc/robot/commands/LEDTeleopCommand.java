package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.lightStrip);
    }
    
    @Override
    public void execute (){

        if(Robot.intake.getIntakeCurrent() > 20 || Robot.lightStrip.triggered){
            Robot.lightStrip.triggered = true;
            Robot.lightStrip.lightTimer.start();
            if(Robot.lightStrip.lightTimer.get() % 0.5 < 0.25){
                Robot.lightStrip.solid(LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
            } else{
                Robot.lightStrip.solid(LEDConstants.GREEN, LEDConstants.WHITE, LEDConstants.FULL);
            }
            if(Robot.lightStrip.lightTimer.get() > 2){
                Robot.lightStrip.triggered = false;
            }
        } else if(Robot.intake.getIntakeCurrent() < 20 && Robot.intake.getIntakeCurrent() > 0){
            Robot.lightStrip.lightTimer.reset();
            Robot.lightStrip.rainbow(Robot.lightStrip.teamRainbow);
        }

        if(Robot.climbers.getClimberEncoder() > 1){
            if(Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight){
                Robot.lightStrip.solid(Robot.lightStrip.offState, LEDConstants.SATURATED, (int)Robot.climbers.getClimberEncoder()*255/ClimberConstants.scoringHeight);
            } else {
                Robot.lightStrip.rainbow(3);
            }
        }

        if(Robot.shooter.getLSpeed() > ShooterConstants.kLArmedRPM - 200){
            Robot.lightStrip.solid(LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
        } else if(Robot.shooter.getLSpeed() > 150){
            Robot.lightStrip.solid(Robot.lightStrip.shooterLo, LEDConstants.SATURATED, LEDConstants.FULL);
        } else{
            Robot.lightStrip.solid(Robot.lightStrip.offState, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    } 
}
