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
        
        if(PlayerConfigs.shooterPrime == true && PlayerConfigs.shooter == true){
            shoot.SetFlywheelVoltage(Constants.ShooterConstants.kShooterVolts);
        }
        else if(PlayerConfigs.shooterPrime == true){
            shoot.SetFlywheelVoltage(Constants.ShooterConstants.kPrimeVolts);
        }
        else{
            shoot.SetFlywheelVoltage(0);
        }
    } 
}
