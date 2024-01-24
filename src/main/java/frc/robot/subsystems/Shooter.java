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

    private final CANSparkMax flywheelL;
    private final CANSparkMax flywheelR;
    private final RelativeEncoder flywheelEncoder;
    private final SparkPIDController flywheelPID;
    public boolean shootingMode;

    public Shooter () {
        flywheelL = new CANSparkMax(CANID.FLYWHEEL_LEFT, MotorType.kBrushless);
        flywheelR = new CANSparkMax(CANID.FLYWHEEL_RIGHT, MotorType.kBrushless);
        flywheelEncoder = flywheelL.getEncoder();
        flywheelPID = flywheelL.getPIDController();

        flywheelR.follow(flywheelL,true);

        flywheelPID.setP(ShooterConstants.kShooterP);

        flywheelL.setIdleMode(IdleMode.kCoast);
        flywheelR.setIdleMode(IdleMode.kCoast);

        flywheelL.burnFlash();
        flywheelR.burnFlash();

        shootingMode = true;
    }

    public double getSpeed(){
            return flywheelEncoder.getVelocity();
    }

    public void SetFlywheelSpeed(double speed) {
        flywheelPID.setReference(speed, ControlType.kVelocity);
    }

    public void shooterLog(){
        Logger.info("SHOOT", Double.toString(getSpeed()) + " RPM");
        if(flywheelL.getFaults()!=0){Logger.warn("FWLFT: " + Short.toString(flywheelL.getFaults()));}
        if(flywheelR.getFaults()!=0){Logger.warn("FWRGT: " + Short.toString(flywheelR.getFaults()));}
    }

}
