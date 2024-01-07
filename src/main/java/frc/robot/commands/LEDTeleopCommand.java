package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.util.Logger;
import frc.robot.Robot;

public class LEDTeleopCommand extends Command{
    
    public LEDTeleopCommand(){
        addRequirements(Robot.ledSystem);
    }
    
    @Override
    public void execute (){
        if (Robot.limelight.getTV() == 1 & 
            (PlayerConfigs.armControl == 180 || PlayerConfigs.align)){
            if (Math.abs(Robot.limelight.getTX()) < 2){
                Robot.ledSystem.solid(60,255,255);
            } else {
                Robot.ledSystem.solid(15,255,50);
            }
        } else if (PlayerConfigs.armControl == 0 || PlayerConfigs.armControl == 90) {
            if(Robot.exampleArm.getIntakeCurrent() > 20){
                Robot.ledSystem.solid(90,255,255);
                Logger.info("INTKE", "TARGET ACQUIRED");
            } else {
                Robot.ledSystem.rainbow();
                Logger.info("INTKE", "COLLECTING...");
            }
        } else {
            Robot.ledSystem.bars(0,120,0);
        }
    } 
}
