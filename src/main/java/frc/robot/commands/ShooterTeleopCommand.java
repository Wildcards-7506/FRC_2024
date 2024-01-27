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

        //If in shooting mode and intake is in shooting position, high speed
        if(Robot.shooter.shootingMode && PlayerConfigs.armScoringMechanism){
            Robot.shooter.setShooterSpeed(ShooterConstants.kLArmedRPM, ShooterConstants.kRArmedRPM);
        //If in shooting mode but not actively shooting, idle speed
        } else if(Robot.shooter.shootingMode){
            Robot.shooter.setShooterSpeed(ShooterConstants.kPrimeRPM, ShooterConstants.kPrimeRPM);
        //If climbing, shooter stops
        } else {
            Robot.shooter.cease();
        }

        SmartDashboard.putNumber("Shooter Speed", Robot.shooter.getRSpeed());
        SmartDashboard.putBoolean("Shooter Mode", Robot.shooter.shootingMode);
        Robot.shooter.shooterLog();
    } 
}
