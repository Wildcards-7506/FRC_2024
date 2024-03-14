package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.Logger;
public class Climbers extends SubsystemBase{
    private CANSparkMax climberRight;
    private CANSparkMax climberLeft;
    
    private RelativeEncoder climbLEncoder;
    private RelativeEncoder climbREncoder;
    public boolean splitControlMode = false;;
    
    public Climbers () {
        climberRight = new CANSparkMax(CANID.CLIMBER_RIGHT, MotorType.kBrushless);
        climberLeft = new CANSparkMax(CANID.CLIMBER_LEFT, MotorType.kBrushless);

        climbLEncoder = climberLeft.getEncoder();
        climbREncoder = climberRight.getEncoder();
        climbLEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderDistancePerPulse);
        climbREncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderDistancePerPulse);
    
        climberLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climberRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        climberRight.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climberLeft.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        climberRight.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);

        climberLeft.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.downLimit);
        climberLeft.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.upLimit);
        climberRight.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.downLimit);
        climberRight.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.upLimit);

        climberLeft.setInverted(true);
        // climberRight.follow(climberLeft, false);

        climberLeft.setOpenLoopRampRate(1);
        climberRight.setOpenLoopRampRate(1);

        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);

        climberLeft.burnFlash();
        climberRight.burnFlash();
    }

    public double getClimberEncoder() {
        return climbLEncoder.getPosition();
    }

    public void setClimbers (double lVolts, double rVolts) {
        climberLeft.setVoltage(lVolts);
        climberRight.setVoltage(rVolts);
    }

    public void climberLog(){
        Logger.info("CLIMB", Double.toString(getClimberEncoder()) + " Inches");
        if(climberLeft.getFaults()!=0){Logger.warn("CLBLT: " + Short.toString(climberLeft.getFaults()));}
        if(climberRight.getFaults()!=0){Logger.warn("CLBRT: " + Short.toString(climberRight.getFaults()));}
    }
        
    
}