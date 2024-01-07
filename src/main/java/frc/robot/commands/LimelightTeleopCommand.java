package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.util.Logger;

public class LimelightTeleopCommand extends Command{

    private boolean prev_StartButton = false;

    public LimelightTeleopCommand(){
        addRequirements(Robot.limelight);
    }

    @Override
    public void execute(){
        Robot.limelight.updateData();
        if(PlayerConfigs.switchPipeline != prev_StartButton){
            prev_StartButton = PlayerConfigs.switchPipeline;
            if(PlayerConfigs.switchPipeline){
                if(Robot.limelight.getPipeline() == 0){
                    Robot.limelight.APipeline();
                    Logger.info("LIGHT", "Switch To A");
                } else {
                    Robot.limelight.BPipeline();
                    Logger.info("LIGHT", "Switch To B");
                }
            }
        }
    }   
}