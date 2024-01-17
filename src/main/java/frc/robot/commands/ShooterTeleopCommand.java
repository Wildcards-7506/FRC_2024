package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ShooterTeleopCommand extends Command{

    private boolean prev_PrimeButton = false;

    public ShooterTeleopCommand(){
        addRequirements(Robot.shooter);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.shooterPrimed != prev_PrimeButton){
            prev_PrimeButton = PlayerConfigs.shooterPrimed;
            if(PlayerConfigs.shooterPrimed){
                Robot.shooter.shootingMode = !Robot.shooter.shootingMode;
            }
        }

        if(PlayerConfigs.shooterPrimed == true && PlayerConfigs.shooterArmed == true){
            Robot.shooter.SetFlywheelSpeed(Constants.ShooterConstants.kArmedRPM);
        } else if(PlayerConfigs.shooterPrimed == true){
            Robot.shooter.SetFlywheelSpeed(Constants.ShooterConstants.kPrimeRPM);
        } else {
            Robot.shooter.SetFlywheelSpeed(0);
        }

        SmartDashboard.putNumber("Shooter Speed", Robot.shooter.getSpeed());
        SmartDashboard.putBoolean("Shooter Mode", Robot.shooter.shootingMode);
    } 
}
