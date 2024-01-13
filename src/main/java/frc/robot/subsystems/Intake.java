package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.util.Logger;

public class Intake extends SubsystemBase{
    private CANSparkMax elbowRotatorLeader;
    private CANSparkMax elbowRotatorFollower;
    private CANSparkMax wristRotator;
    private CANSparkMax intakeLeader;
    private CANSparkMax intakeFollower;
    
    private RelativeEncoder elbowLEncoder;
    private RelativeEncoder elbowFEncoder;
    private RelativeEncoder wristEncoder;

    private SparkPIDController elbowPID;
    private SparkPIDController wristPID;

    public Intake() {
        elbowRotatorLeader = new CANSparkMax(CANID.ELBOW_LEFT, MotorType.kBrushless);
        elbowRotatorFollower = new CANSparkMax(CANID.ELBOW_RIGHT, MotorType.kBrushless);
        wristRotator = new CANSparkMax(CANID.WRIST, MotorType.kBrushless);
        intakeLeader = new CANSparkMax(CANID.INTAKE_LEFT, MotorType.kBrushless);
        intakeFollower = new CANSparkMax(CANID.INTAKE_RIGHT, MotorType.kBrushless);

        elbowLEncoder = elbowRotatorLeader.getEncoder();
        elbowFEncoder = elbowRotatorFollower.getEncoder();
        wristEncoder = wristRotator.getEncoder();

        elbowPID = elbowRotatorLeader.getPIDController();
        wristPID = wristRotator.getPIDController();

        elbowRotatorFollower.follow(elbowRotatorLeader, true);
        intakeFollower.follow(intakeLeader, true);

        configureMotorLimits();
        configureEncoders();
        configurePID();

        elbowRotatorLeader.burnFlash();
        elbowRotatorFollower.burnFlash();
        wristRotator.burnFlash();
        intakeLeader.burnFlash();
        intakeFollower.burnFlash();
    }

    private void configureMotorLimits() {
        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kForward, true);
        elbowRotatorFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
        wristRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);

        intakeLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
        intakeLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);
        intakeFollower.enableSoftLimit(SoftLimitDirection.kForward, false);
        intakeFollower.enableSoftLimit(SoftLimitDirection.kReverse, false);

        elbowRotatorLeader.setSmartCurrentLimit(Constants.ElbowConstants.kElbowCurrentLimit);
        elbowRotatorFollower.setSmartCurrentLimit(Constants.ElbowConstants.kElbowCurrentLimit);
        wristRotator.setSmartCurrentLimit(Constants.WristConstants.kWristCurrentLimit);
        intakeLeader.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeCurrentLimit);
        intakeFollower.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeCurrentLimit);
    }

    private void configureEncoders() {
        elbowLEncoder.setPositionConversionFactor(Constants.ElbowConstants.kElbowEncoderDistancePerPulse);
        elbowFEncoder.setPositionConversionFactor(Constants.ElbowConstants.kElbowEncoderDistancePerPulse);
        wristEncoder.setPositionConversionFactor(Constants.WristConstants.kWristEncoderDistancePerPulse);
    }

    private void configurePID() {
        elbowPID.setP(Constants.ElbowConstants.kElbowKP);
        wristPID.setP(Constants.WristConstants.kWristKP);

        elbowPID.setOutputRange(-1, 1);
        wristPID.setOutputRange(-1, 1);
    }

    public double getElbowLEncoder() {
        return elbowLEncoder.getPosition();
    }

    public double getElbowFEncoder() {
        return elbowFEncoder.getPosition();
    }

    public double getWristEncoder() {
        return wristEncoder.getPosition();
    }
    
    // Code here comes from FRC 2023 I just modified it for 2024
    // What is the meaning of this arbFF???? I'm about to barf on em.
    // What is elbowHorizontalOffset? Is it for the initial offset of the old crane arm 2023?
    public void setElbowPosition(double setPoint) {
        double arbFF = 0 * Math.cos(Math.toRadians(getElbowLEncoder() - Constants.ElbowConstants.elbowHorizontalOffset));
        elbowPID.setReference(setPoint, CANSparkMax.ControlType.kPosition, 0, arbFF);
        SmartDashboard.putNumber("Arm Setpoint", setPoint);
    }

	public void setWristPosition(double setPoint) {
        wristPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    public void errorCheck(){
        if(elbowRotatorLeader.getFaults()!=0){Logger.warn("ELBWL: " + Short.toString(elbowRotatorLeader.getFaults()));}
        if(elbowRotatorFollower.getFaults()!=0){Logger.warn("ELBWF: " + Short.toString(elbowRotatorFollower.getFaults()));}
        if(wristRotator.getFaults()!=0){Logger.warn("WRIST: " + Short.toString(wristRotator.getFaults()));}
        if(intakeLeader.getFaults()!=0){Logger.warn("INTKL: " + Short.toString(intakeLeader.getFaults()));}
        if(intakeFollower.getFaults()!=0){Logger.warn("INTKF: " + Short.toString(intakeFollower.getFaults()));}
    }
}
