package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;
import frc.robot.subsystems.Shooter;

public class ShooterTeleopCommand extends Command{

    Shooter shoot = new Shooter();

    public ShooterTeleopCommand(){
        addRequirements(Robot.shooter);
    }
    
    @Override
    public void execute (){
        //Need to set Shooter Primed as a boolean toggle to turn climbers on/off and use lights

        if(PlayerConfigs.shooterPrimed == true && PlayerConfigs.shooterArmed == true){
            shoot.SetFlywheelSpeed(Constants.ShooterConstants.kArmedRPM);
        } else if(PlayerConfigs.shooterPrimed == true){
            shoot.SetFlywheelSpeed(Constants.ShooterConstants.kPrimeRPM);
        } else {
            shoot.SetFlywheelSpeed(0);
        }

        //Add code to post shooter speed to the dashboard. See ClimberTeleopCommand.java for an example
    } 
}
