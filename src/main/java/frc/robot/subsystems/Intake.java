package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CANID;
import frc.robot.util.Logger;

public class Intake extends SubsystemBase {
    private CANSparkMax elbowRotatorLeader;
    private CANSparkMax elbowRotatorFollower;
    private CANSparkMax wristRotator;
    private CANSparkMax intake;
    
    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;

    public SparkPIDController elbowPIDF;
    public SparkPIDController wristPIDF;

    public double wristSetPoint;
    public double elbowSetPoint;
    public int intakeState = 0; //<- CHANGE THIS TO ZERO BEFORE COMPS
    public boolean fcControlElbow;
    public boolean fcControlWrist;

    public Intake() {
        elbowRotatorLeader = new CANSparkMax(CANID.ELBOW_LEFT, MotorType.kBrushless);
        elbowRotatorFollower = new CANSparkMax(CANID.ELBOW_RIGHT, MotorType.kBrushless);
        wristRotator = new CANSparkMax(CANID.WRIST, MotorType.kBrushless);
        intake = new CANSparkMax(CANID.INTAKE, MotorType.kBrushless);

        elbowRotatorLeader.setIdleMode(IdleMode.kCoast);
        elbowRotatorFollower.setIdleMode(IdleMode.kCoast);
        wristRotator.setIdleMode(IdleMode.kCoast);
        intake.setIdleMode(IdleMode.kCoast);

        elbowEncoder = elbowRotatorLeader.getEncoder();
        wristEncoder = wristRotator.getEncoder();

        elbowPIDF = elbowRotatorLeader.getPIDController();
        wristPIDF = wristRotator.getPIDController();

        elbowRotatorFollower.follow(elbowRotatorLeader, true);

        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        wristRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Will need to test these angle parameters when testing
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kForward, 160);
        elbowRotatorLeader.setSoftLimit(SoftLimitDirection.kReverse, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kForward, 0);
        elbowRotatorFollower.setSoftLimit(SoftLimitDirection.kReverse, -160);
        wristRotator.setSoftLimit(SoftLimitDirection.kForward, 170);
        wristRotator.setSoftLimit(SoftLimitDirection.kReverse, -80);

        elbowRotatorLeader.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        elbowRotatorFollower.setSmartCurrentLimit(IntakeConstants.kElbowCurrentLimit);
        wristRotator.setSmartCurrentLimit(IntakeConstants.kWristCurrentLimit);
        intake.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);

        elbowEncoder.setPositionConversionFactor(IntakeConstants.kElbowEncoderDistancePerPulse);
        wristEncoder.setPositionConversionFactor(IntakeConstants.kWristEncoderDistancePerPulse);

        elbowPIDF.setP(IntakeConstants.kPElbow);
        wristPIDF.setP(IntakeConstants.kPWrist);

        elbowPIDF.setOutputRange(-1, 1);
        wristPIDF.setOutputRange(-1, 1);

        elbowRotatorLeader.burnFlash();
        elbowRotatorFollower.burnFlash();
        wristRotator.burnFlash();
        intake.burnFlash();
    }

    public double getElbowEncoder() {
        return elbowEncoder.getPosition();
    }

    public double getWristEncoder() {
        return wristEncoder.getPosition();
    }

    public double getIntakeCurrent() {
        return intake.getOutputCurrent();
    }
    
    public void setElbowPosition(double setPoint) {
        elbowPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

	public void setWristPosition(double setPoint) {
        wristPIDF.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

    public void setIntakeVoltage(double setPoint) {
        intake.setVoltage(setPoint);
    }

    public void intakeLog(){
        Logger.info("ELBOW", Double.toString(getElbowEncoder()) + " Actual Degrees -> " + Double.toString(elbowSetPoint) + " Target Degrees");
        Logger.info("WRIST", Double.toString(getWristEncoder()) + " Actual Degrees -> " + Double.toString(wristSetPoint) + " Target Degrees");
        if(elbowRotatorLeader.getFaults()!=0){Logger.warn("ELBWL: " + Short.toString(elbowRotatorLeader.getFaults()));}
        if(elbowRotatorFollower.getFaults()!=0){Logger.warn("ELBWF: " + Short.toString(elbowRotatorFollower.getFaults()));}
        if(wristRotator.getFaults()!=0){Logger.warn("WRIST: " + Short.toString(wristRotator.getFaults()));}
        if(intake.getFaults()!=0){Logger.warn("INTKL: " + Short.toString(intake.getFaults()));}
    }
}
