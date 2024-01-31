package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Logger;

public class Shooter extends SubsystemBase{

    private final CANSparkMax shooterL;
    private final CANSparkMax shooterR;
    private final RelativeEncoder shooterLEncoder;
    private final SparkPIDController shooterLPID;
    private final RelativeEncoder shooterREncoder;
    private final SparkPIDController shooterRPID;
    public boolean shootingMode;

    public Shooter () {
        shooterL = new CANSparkMax(CANID.shooter_LEFT, MotorType.kBrushless);
        shooterR = new CANSparkMax(CANID.shooter_RIGHT, MotorType.kBrushless);

        shooterLEncoder = shooterL.getEncoder();
        shooterLPID = shooterL.getPIDController();
        shooterREncoder = shooterR.getEncoder();
        shooterRPID = shooterR.getPIDController();

        shooterL.setInverted(true);

        shooterLPID.setP(ShooterConstants.kShooterP);
        shooterRPID.setP(ShooterConstants.kShooterP);

        shooterL.setIdleMode(IdleMode.kCoast);
        shooterR.setIdleMode(IdleMode.kCoast);

        shooterL.burnFlash();
        shooterR.burnFlash();

        shootingMode = true;
    }

    public double getRSpeed(){
            return shooterREncoder.getVelocity();
    }

    public double getLSpeed(){
            return shooterLEncoder.getVelocity();
    }

    public void setShooterSpeed(double lSpeed, double rSpeed) {
        shooterLPID.setReference(lSpeed, ControlType.kVelocity);
        shooterRPID.setReference(rSpeed, ControlType.kVelocity);
    }
    
    public void shooterLog(){
        Logger.info("SHOOT", "Right: " + Double.toString(getRSpeed()) + " RPM, " + "Left: " + Double.toString(getLSpeed()) + " RPM");
        if(shooterL.getFaults()!=0){Logger.warn("FWLFT: " + Short.toString(shooterL.getFaults()));}
        if(shooterR.getFaults()!=0){Logger.warn("FWRGT: " + Short.toString(shooterR.getFaults()));}
    }

}
