package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.ControlConfigs.PlayerConfigs;

public class ShooterTeleopCommand extends Command{

    private boolean prev_ActiveButton = false;
    private boolean prev_ArmScoringMechanism = false;

    public ShooterTeleopCommand(){
        addRequirements(Robot.shooter);
    }
    
    @Override
    public void execute (){
        // primes shooter
        if(PlayerConfigs.shooterActive != prev_ActiveButton){
            prev_ActiveButton = PlayerConfigs.shooterActive;
            if(PlayerConfigs.shooterActive){
                Robot.shooter.shootingMode = !Robot.shooter.shootingMode;
            }
        }
        
        // arms shooter
        if(PlayerConfigs.armScoringMechanism != prev_ArmScoringMechanism){
            prev_ArmScoringMechanism = PlayerConfigs.armScoringMechanism;
            if(PlayerConfigs.armScoringMechanism){
                Robot.shooter.armScoringMode = !Robot.shooter.armScoringMode;
            }
        }
        
        // when primed, shooter can be toggled between prime (false) and armed (true)
        // when not primed, both shooter and prime are disabled (false), and toggle is reset for arming
        if (!Robot.shooter.shootingMode) {
            prev_ArmScoringMechanism = false;
            Robot.shooter.armScoringMode = false;
        }

        //If in shooting mode and intake is in shooting position, high speed
        if(Robot.shooter.shootingMode && Robot.shooter.armScoringMode){
            SmartDashboard.putString("Shooter Status", "DANGER - Armed");
            Robot.shooter.setShooterSpeed(ShooterConstants.kLArmedRPM, ShooterConstants.kRArmedRPM);
        //If in shooting mode but not actively shooting, idle speed
        } else if(Robot.shooter.shootingMode){
            SmartDashboard.putString("Shooter Status", "Idle");
            Robot.shooter.setShooterSpeed(ShooterConstants.kPrimeRPM, ShooterConstants.kPrimeRPM);
        //If climbing, shooter stops
        } else {
            SmartDashboard.putString("Shooter Status", "Disabled");
            Robot.shooter.setShooterSpeed(0,0);
        }

        SmartDashboard.putNumber("Right Shooter Speed", Robot.shooter.getRSpeed());
        SmartDashboard.putNumber("Left Shooter Speed", Robot.shooter.getLSpeed());
        SmartDashboard.putBoolean("Shooter Mode", Robot.shooter.shootingMode);
        Robot.shooter.shooterLog();
    } 
}
