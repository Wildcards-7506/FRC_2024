package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.LEDConstants;
import frc.robot.ControlConfigs.PlayerConfigs;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.ledSystem);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.align){
            if(Math.abs(Robot.limelight.getTX()) < 2){
                Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, LEDConstants.ORANGE, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            }
        } else {
            Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        }

        if(Robot.shooter.getSpeed() > 2800){
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.shooter.getSpeed() > 150){
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, LEDConstants.RED, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else{
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        }

        // if(Robot.intake.pieceAcquired){
        //     Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        // } else if(Robot.intake.running){
        //     Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, LEDConstants.MAGENTA, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        // } else {
        //     Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, LEDConstants.RED, LEDConstants.SV_OFF, LEDConstants.SV_OFF);
        // }

        if(Robot.climbers.getClimberEncoder() > 2){
            if(Robot.climbers.getClimberEncoder() > 22){
                Robot.ledSystem.solid(LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.rainbow();
            }
        }
    } 
}
