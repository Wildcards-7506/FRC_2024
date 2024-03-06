package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ControlConfigs.PlayerConfigs;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.lightStrip);
    }
    
    @Override
    public void execute (){

        if(Robot.intake.getIntakeCurrent() > 20){
            Robot.lightStrip.solid(LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
        } else if(Robot.intake.getIntakeCurrent() < 20 && Robot.intake.getIntakeCurrent() > 0){
            Robot.lightStrip.rainbow(Robot.lightStrip.teamRainbow);
        }

        if(Robot.climbers.getClimberEncoder() > 1){
            if(Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight){
                Robot.lightStrip.solid(Robot.lightStrip.offState, LEDConstants.SATURATED, (int)Robot.climbers.getClimberEncoder()*255/ClimberConstants.scoringHeight);
            } else {
                Robot.lightStrip.rainbow(3);
            }
        }

        if(PlayerConfigs.align){
            if(Math.abs(Robot.limelight.getTX()) < 2){
                Robot.lightStrip.section(0, LEDConstants.bufferSize/2-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
            } else {
                Robot.lightStrip.section(0, LEDConstants.bufferSize/2-1, Robot.lightStrip.alignOOB, LEDConstants.SATURATED, LEDConstants.FULL);
            }
        } else {
            Robot.lightStrip.section(0, LEDConstants.bufferSize/2-1, Robot.lightStrip.offState, LEDConstants.SATURATED, LEDConstants.FULL);
        }

        if(Robot.shooter.getLSpeed() > ShooterConstants.kLArmedRPM - 200){
            Robot.lightStrip.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
        } else if(Robot.shooter.getLSpeed() > 150){
            Robot.lightStrip.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.lightStrip.shooterLo, LEDConstants.SATURATED, LEDConstants.FULL);
        } else{
            Robot.lightStrip.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.lightStrip.offState, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    } 
}
