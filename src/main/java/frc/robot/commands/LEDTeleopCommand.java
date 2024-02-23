package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        int alignOOB = Robot.teamColor.get() == Alliance.Red ? LEDConstants.PINK : LEDConstants.VIOLET;
        int shooterLo = Robot.teamColor.get() == Alliance.Red ? LEDConstants.ORANGE : LEDConstants.AZURE;
        int intakeLo = Robot.teamColor.get() == Alliance.Red ? LEDConstants.YELLOW : LEDConstants.CYAN;
        int offState = Robot.teamColor.get() == Alliance.Red ? LEDConstants.RED : LEDConstants.BLUE;

        if(PlayerConfigs.align){
            if(Math.abs(Robot.limelight.getTX()) < 2){
                Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            } else {
                Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, alignOOB, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
            }
        } else {
            Robot.ledSystem.section(0, LEDConstants.bufferSize/3-1, offState, LEDConstants.SV_FULL, LEDConstants.SV_MID);
        }

        if(Robot.shooter.getLSpeed() > ShooterConstants.kLArmedRPM - 200){
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.shooter.getLSpeed() > 150){
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, shooterLo, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else{
            Robot.ledSystem.section(LEDConstants.bufferSize/3, 2*LEDConstants.bufferSize/3-1, offState, LEDConstants.SV_FULL, LEDConstants.SV_MID);
        }

        if(Robot.intake.getIntakeCurrent() > 20){
            Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, LEDConstants.GREEN, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else if(Robot.intake.getIntakeCurrent() < 20 && Robot.intake.getIntakeCurrent() > 0){
            Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, intakeLo, LEDConstants.SV_FULL, LEDConstants.SV_FULL);
        } else {
            Robot.ledSystem.section(2*LEDConstants.bufferSize/3, LEDConstants.bufferSize-1, offState, LEDConstants.SV_FULL, LEDConstants.SV_MID);
        }

        if(Robot.climbers.getClimberEncoder() > 1){
            if(Robot.climbers.getClimberEncoder() < ClimberConstants.scoringHeight){
                Robot.ledSystem.solid(offState, LEDConstants.SV_FULL, (int)Robot.climbers.getClimberEncoder()*255/ClimberConstants.scoringHeight);
            } else {
                Robot.ledSystem.rainbow();
            }
        }
    } 
}
