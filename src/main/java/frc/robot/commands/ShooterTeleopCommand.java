package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ShooterTeleopCommand extends Command{

    private boolean prev_ActiveButton = false;

    public ShooterTeleopCommand(){
        addRequirements(Robot.shooter);
    }
    
    @Override
    public void execute (){
        if(PlayerConfigs.shooterActive != prev_ActiveButton){
            prev_ActiveButton = PlayerConfigs.shooterActive;
            if(PlayerConfigs.shooterActive){
                Robot.shooter.shootingMode = !Robot.shooter.shootingMode;
            }
        }

        if(Robot.shooter.shootingMode == true && PlayerConfigs.armScoringMechanism == true){
            Robot.shooter.SetshooterSpeed(ShooterConstants.kLArmedRPM, ShooterConstants.kRArmedRPM);
        } else if(Robot.shooter.shootingMode == true){
            Robot.shooter.SetshooterSpeed(ShooterConstants.kPrimeRPM, ShooterConstants.kPrimeRPM);
        } else {
            Robot.shooter.SetshooterSpeed(0,0);
        }

        SmartDashboard.putNumber("Shooter Speed", Robot.shooter.getRSpeed());
        SmartDashboard.putBoolean("Shooter Mode", Robot.shooter.shootingMode);
        Robot.shooter.shooterLog();
    } 
}
