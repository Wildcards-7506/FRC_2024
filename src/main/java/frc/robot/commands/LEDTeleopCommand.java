package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ControlConfigs.PlayerConfigs;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.ledSystem);
    }
    
    @Override
    public void execute (){

        if(Robot.intake.getIntakeCurrent() > 20){
            Robot.ledSystem.solid(LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.intake.getIntakeCurrent() < 20 && Robot.intake.getIntakeCurrent() > 0){
            Robot.ledSystem.rainbow(Robot.ledSystem.teamRainbow);
        }

        if(Robot.climbers.getClimberEncoder() > 1){
            if(Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight){
                Robot.ledSystem.solid(Robot.ledSystem.offState, LEDConstants.SV_FULL, (int)Robot.climbers.getClimberEncoder()*255/ClimberConstants.scoringHeight);
            } else {
                Robot.ledSystem.rainbow(3);
            }
        }

        if(PlayerConfigs.align){
            if(Math.abs(Robot.limelight.getTX()) < 2){
                Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, Robot.ledSystem.alignOOB, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            }
        } else {
            Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, Robot.ledSystem.offState, LEDConstants.SV_FULL, LEDConstants.SV_MID);
        }

        if(Robot.shooter.getLSpeed() > ShooterConstants.kLArmedRPM - 200){
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.shooter.getLSpeed() > 150){
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.ledSystem.shooterLo, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else{
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.ledSystem.offState, LEDConstants.SV_FULL, LEDConstants.SV_MID);
        }
    } 
}
