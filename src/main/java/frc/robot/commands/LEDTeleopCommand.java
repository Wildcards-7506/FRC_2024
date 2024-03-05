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
        if(PlayerConfigs.align){
            if(Robot.intake.pieceAcquired){
                Robot.ledSystem.solid(LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
            } else if(Robot.intake.running){
                Robot.ledSystem.rainbow(Robot.ledSystem.teamRainbow);
            }
        }

        if(Robot.climbers.getClimberEncoder() > 1){
            if(Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight){
                Robot.ledSystem.solid(Robot.ledSystem.offState, LEDConstants.SATURATED, (int)Robot.climbers.getClimberEncoder()*255/ClimberConstants.scoringHeight);
            } else {
                Robot.ledSystem.rainbow(3);
            }
        }
        
        if(PlayerConfigs.align){
            if(Math.abs(Robot.limelight.getTX()) < 2){
                Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
            } else {
                Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, Robot.ledSystem.alignOOB, LEDConstants.SATURATED, LEDConstants.FULL);
            }
        } else {
            Robot.ledSystem.section(0, LEDConstants.bufferSize/2-1, Robot.ledSystem.offState, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    
        if(Robot.shooter.getSpeed() > ShooterConstants.kArmedRPM - 200){
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, LEDConstants.GREEN, LEDConstants.SATURATED, LEDConstants.FULL);
        } else if(Robot.shooter.getSpeed() > 150){
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.ledSystem.shooterLo, LEDConstants.SATURATED, LEDConstants.FULL);
        } else{
            Robot.ledSystem.section(LEDConstants.bufferSize/2, LEDConstants.bufferSize-1, Robot.ledSystem.offState, LEDConstants.SATURATED, LEDConstants.FULL);
        }
    } 
}
