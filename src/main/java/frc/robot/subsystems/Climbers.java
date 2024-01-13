package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ExampleArmConstants;
import frc.robot.util.Logger;
public class Climbers extends SubsystemBase{
    private CANSparkMax climberright;
    private CANSparkMax climberleft;
    
    private RelativeEncoder climbEncoder;
    public Climbers () {
        climberright = new CANSparkMax(CANID.CLIMBER_RIGHT, MotorType.kBrushless);
        climberleft = new CANSparkMax(CANID.CLIMBER_LEFT, MotorType.kBrushless);

        climbEncoder = climberleft.getEncoder();
        climbEncoder.setPositionConversionFactor(ClimberConstants.kclimberEncoderDistancePerPulse);
    
        climberleft.enableSoftLimit(SoftLimitDirection.kForward, true);
        climberleft.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climberright.enableSoftLimit(SoftLimitDirection.kForward, true);
        climberright.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climberleft.setSmartCurrentLimit(ClimberConstants.kclimberCurrentLimit);
        climberright.setSmartCurrentLimit(ClimberConstants.kclimberCurrentLimit);

        climberleft.setSoftLimit(SoftLimitDirection.kForward, 24);
        climberleft.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climberright.setSoftLimit(SoftLimitDirection.kForward, 24);
        climberright.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climberleft.burnFlash();
        climberright.burnFlash();}

         public double getClimberEncoder() {
            return climbEncoder.getPosition();
        }

        public void setclimbers (double volts) {
            climberleft.setVoltage(volts);
            climberright.setVoltage(volts);
        }

        
    
}