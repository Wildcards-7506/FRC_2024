package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

    public SparkPIDController climberLPIDF;
    public SparkPIDController climberRPIDF;

    public boolean climberEngage = false;
    
    public Climbers () {
        climberRight = new CANSparkMax(CANID.CLIMBER_RIGHT, MotorType.kBrushless);
        climberLeft = new CANSparkMax(CANID.CLIMBER_LEFT, MotorType.kBrushless);

        climbLEncoder = climberLeft.getEncoder();
        climbREncoder = climberRight.getEncoder();
        climbLEncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderDistancePerPulse);
        climbREncoder.setPositionConversionFactor(ClimberConstants.kClimberEncoderDistancePerPulse);

        climberLPIDF = climberLeft.getPIDController();
        climberRPIDF = climberRight.getPIDController();

        climberLPIDF.setP(ClimberConstants.kPClimber);
        climberRPIDF.setP(ClimberConstants.kPClimber);
    
        climberLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
        climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
        climberRight.enableSoftLimit(SoftLimitDirection.kForward, false);
        climberRight.enableSoftLimit(SoftLimitDirection.kReverse, false);

        climberLeft.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        climberRight.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);

        climberLeft.setSoftLimit(SoftLimitDirection.kForward, 125);
        climberLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climberRight.setSoftLimit(SoftLimitDirection.kForward, 125);
        climberRight.setSoftLimit(SoftLimitDirection.kReverse, 0);

        climberLeft.setInverted(false);
        climberRight.setInverted(true);

        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);

        climberLPIDF.setOutputRange(-1.0, 1.0);
        climberRPIDF.setOutputRange(-1.0, 1.0);

        climberLeft.burnFlash();
        climberRight.burnFlash();
    }

    public double getClimberREncoder() {
        return climbREncoder.getPosition();
    }

    public double getClimberLEncoder() {
        return climbLEncoder.getPosition();
    }

    public void setClimbers (double setpoint) {
        climberLPIDF.setReference(setpoint, ControlType.kPosition);
        climberRPIDF.setReference(setpoint, ControlType.kPosition);
    }

    public void climberLog(){
        Logger.info("CLMBR", Double.toString(getClimberREncoder()) + " Inches");
        Logger.info("CLMBL", Double.toString(getClimberLEncoder()) + " Inches");
        if(climberLeft.getFaults()!=0){Logger.warn("CLMBL: " + Short.toString(climberLeft.getFaults()));}
        if(climberRight.getFaults()!=0){Logger.warn("CLMBR: " + Short.toString(climberRight.getFaults()));}
    }
        
    
}